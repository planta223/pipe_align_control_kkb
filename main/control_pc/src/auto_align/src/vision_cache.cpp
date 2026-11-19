#include "auto_align/vision_align.hpp"
#include <chrono>
#include <fstream>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <Eigen/SVD>

namespace auto_align {
namespace {
void require(bool ok,const std::string& message) { if(!ok) throw std::runtime_error(message); }
double wall_time() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}
template<class Matrix> YAML::Node matrix_node(const Matrix& matrix) {
  YAML::Node out;
  for(int row=0;row<matrix.rows();++row) {
    YAML::Node values;
    for(int col=0;col<matrix.cols();++col) values.push_back(matrix(row,col));
    out.push_back(values);
  }
  return out;
}
template<class Matrix> void read_matrix(const YAML::Node& node,Matrix& matrix) {
  require(node.IsSequence() && node.size()==static_cast<size_t>(matrix.rows()),"cache matrix rows");
  for(int row=0;row<matrix.rows();++row) {
    require(node[row].IsSequence() && node[row].size()==static_cast<size_t>(matrix.cols()),"cache matrix columns");
    for(int col=0;col<matrix.cols();++col) matrix(row,col)=node[row][col].as<double>();
  }
  require(matrix.allFinite(),"cache matrix finite");
}
void validate(const VisionCacheEntry& entry,bool rotation,const VisionSettings& s) {
  require(entry.jacobian.allFinite() && entry.reference.endpoint.allFinite() && entry.reference.axis.allFinite() &&
    std::abs(entry.reference.axis.norm()-1)<1e-6 && entry.tcp.matrix().allFinite() &&
    (entry.tcp.matrix().row(3)-Eigen::RowVector4d(0,0,0,1)).norm()<1e-9 &&
    (entry.tcp.linear().transpose()*entry.tcp.linear()-Eigen::Matrix3d::Identity()).norm()<1e-6 &&
    std::abs(entry.tcp.linear().determinant()-1)<1e-6 && std::isfinite(entry.saved_at) &&
    std::isfinite(entry.position_error) && entry.position_error>=0 &&
    std::isfinite(entry.angle_error) && entry.angle_error>=0,"cache geometry");
  if(rotation) visual_solve(entry.jacobian.bottomRows<2>(),Eigen::Vector2d::Zero(),1,s.damping,s.max_rotation);
  else visual_solve(entry.jacobian.topRows<3>(),Eigen::Vector3d::Zero(),1,s.damping,s.max_translation);
}
}
VisionCache::VisionCache(std::filesystem::path path,std::string identity,VisionSettings settings)
  :path_(std::move(path)),identity_(std::move(identity)),settings_(std::move(settings)) {}
std::string VisionCache::load(bool force_relearn) {
  entries_={};
  if(!settings_.cache_enabled) return "CACHE: disabled";
  if(force_relearn) return "CACHE: force_relearn";
  try {
    if(!std::filesystem::exists(path_)) return "CACHE: missing / 신규 학습";
    require(std::filesystem::file_size(path_)<=65536,"cache file too large");
    const auto root=YAML::LoadFile(path_.string());
    require(root["schema"].as<int>()==1 && root["identity"].as<std::string>()==identity_,"version/context mismatch");
    int loaded=0,rejected=0;
    for(size_t side=0;side<2;++side) for(size_t kind=0;kind<2;++kind) {
      try {
        const auto entries=root["entries"];if(!entries) continue;
        const auto arm=entries[side==0?"left":"right"];if(!arm) continue;
        const auto node=arm[kind==0?"translation":"rotation"];if(!node) continue;
        VisionCacheEntry e;
        read_matrix(node["jacobian"],e.jacobian);read_matrix(node["endpoint_m"],e.reference.endpoint);
        read_matrix(node["axis"],e.reference.axis);read_matrix(node["base_T_tcp"],e.tcp.matrix());
        e.saved_at=node["saved_at_unix_s"].as<double>();
        e.position_error=node["quality"]["position_uncertainty_m"].as<double>();
        e.angle_error=node["quality"]["angle_uncertainty_rad"].as<double>();validate(e,kind==1,settings_);
        const double age=wall_time()-e.saved_at;
        require(age>=0 && age<=settings_.cache_max_age,"expired cache");
        entries_[side][kind]=e;++loaded;
      } catch(const std::exception&) { ++rejected; }
    }
    return "CACHE: loaded="+std::to_string(loaded)+" rejected="+std::to_string(rejected)+" / 이동 응답 검증 전";
  } catch(const std::exception& e) {
    entries_={};return std::string("CACHE: ignored / ")+e.what();
  }
}
void VisionCache::save(const VisionCacheEntries& entries) {
  if(!settings_.cache_enabled) return;
  YAML::Node root;root["schema"]=1;root["identity"]=identity_;
  root["features"]="camera_endpoint_m, signed_atan2_xy_xz_rad; input=base_tcp_m_or_rad";
  for(size_t side=0;side<2;++side) for(size_t kind=0;kind<2;++kind) if(entries[side][kind]) {
    const auto& e=*entries[side][kind];validate(e,kind==1,settings_);
    auto n=root["entries"][side==0?"left":"right"][kind==0?"translation":"rotation"];
    n["jacobian"]=matrix_node(e.jacobian);n["endpoint_m"]=matrix_node(e.reference.endpoint);
    n["axis"]=matrix_node(e.reference.axis);n["base_T_tcp"]=matrix_node(e.tcp.matrix());
    n["saved_at_unix_s"]=e.saved_at;
    const Eigen::MatrixXd task=kind==1?Eigen::MatrixXd(e.jacobian.bottomRows<2>()):Eigen::MatrixXd(e.jacobian.topRows<3>());
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(task);
    n["quality"]["task_sigma_min"]=svd.singularValues().minCoeff();
    n["quality"]["task_condition"]=svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
    n["quality"]["position_uncertainty_m"]=e.position_error;
    n["quality"]["angle_uncertainty_rad"]=e.angle_error;
  }
  YAML::Emitter output;output.SetDoublePrecision(17);output<<root;
  require(output.good(),"cache serialization failed");
  std::filesystem::create_directories(path_.parent_path());
  std::string temporary=path_.string()+".tmp.XXXXXX";
  const int fd=mkstemp(temporary.data());require(fd>=0,"cache temporary file failed");
  bool open=true;
  try {
    const std::string data=output.c_str();size_t written=0;
    while(written<data.size()) {
      const auto count=::write(fd,data.data()+written,data.size()-written);
      if(count<0 && errno==EINTR) continue;
      require(count>0,std::string("cache write: ")+std::strerror(errno));written+=static_cast<size_t>(count);
    }
    require(fsync(fd)==0,"cache fsync failed");
    const int closed=::close(fd);open=false;require(closed==0,"cache close failed");
    std::filesystem::rename(temporary,path_);entries_=entries;
  } catch(...) {
    if(open) ::close(fd);
    ::unlink(temporary.c_str());throw;
  }
}
} // namespace auto_align
