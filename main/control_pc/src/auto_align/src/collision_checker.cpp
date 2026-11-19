#include "auto_align/collision_checker.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <fcl/fcl.h>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <cmath>
#include <filesystem>
#include <map>
#include <iomanip>
#include <sstream>

namespace auto_align {
namespace {
using Geometry = fcl::CollisionGeometryd;
void require(bool condition, const std::string& text) {
  if (!condition) throw std::runtime_error(text);
}
void valid_pose(const xm::Pose& value) {
  require(value.matrix().allFinite() &&
    (value.linear().transpose()*value.linear()-Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
    std::abs(value.linear().determinant()-1) < 1e-6 &&
    (value.matrix().row(3)-Eigen::RowVector4d(0,0,0,1)).norm() < 1e-9,
    "충돌 형상에 올바른 강체 변환이 필요합니다.");
}
bool finite_aabb(const fcl::AABBd& box) { return box.min_.allFinite() && box.max_.allFinite(); }
Eigen::Vector3d vector3(const YAML::Node& node) {
  require(node.IsSequence() && node.size() == 3,"수치 3개가 필요합니다.");
  Eigen::Vector3d result(node[0].as<double>(),node[1].as<double>(),node[2].as<double>());
  require(result.allFinite(),"유한한 수치가 필요합니다.");
  return result;
}
double number(const YAML::Node& node) {
  const double value = node.as<double>();
  require(std::isfinite(value),"유한한 환경 수치가 필요합니다.");
  return value;
}
Eigen::Matrix3d rpy(const Eigen::Vector3d& angles) {
  return (Eigen::AngleAxisd(angles.z(),Eigen::Vector3d::UnitZ())*
    Eigen::AngleAxisd(angles.y(),Eigen::Vector3d::UnitY())*
    Eigen::AngleAxisd(angles.x(),Eigen::Vector3d::UnitX())).toRotationMatrix();
}
xm::Pose pose(const urdf::Pose& value) {
  xm::Pose result = xm::Pose::Identity();
  result.translation() = Eigen::Vector3d(value.position.x,value.position.y,value.position.z);
  result.linear() = Eigen::Quaterniond(value.rotation.w,value.rotation.x,value.rotation.y,value.rotation.z).toRotationMatrix();
  return result;
}
xm::Pose pose(const KDL::Frame& value) {
  xm::Pose result = xm::Pose::Identity();
  for (int row = 0; row < 3; ++row) {
    result.translation()[row] = value.p[row];
    for (int col = 0; col < 3; ++col) result.linear()(row,col) = value.M(row,col);
  }
  return result;
}
std::string mesh_path(const std::string& uri) {
  if (uri.rfind("package://",0) == 0) {
    const auto split = uri.find('/',10);
    require(split != std::string::npos,"메시 package URI가 잘못됐습니다.");
    return ament_index_cpp::get_package_share_directory(uri.substr(10,split-10))+uri.substr(split);
  }
  if (uri.rfind("file://",0) == 0) return uri.substr(7);
  require(std::filesystem::path(uri).is_absolute(),"메시는 package URI 또는 절대 경로여야 합니다.");
  return uri;
}
std::shared_ptr<Geometry> geometry(const urdf::GeometrySharedPtr& shape) {
  require(bool(shape),"충돌 형상이 없습니다.");
  if (shape->type == urdf::Geometry::BOX) {
    auto box = std::static_pointer_cast<urdf::Box>(shape);
    require(std::isfinite(box->dim.x) && std::isfinite(box->dim.y) && std::isfinite(box->dim.z) &&
      box->dim.x > 0 && box->dim.y > 0 && box->dim.z > 0,"box 크기가 잘못됐습니다.");
    return std::make_shared<fcl::Boxd>(box->dim.x,box->dim.y,box->dim.z);
  }
  if (shape->type == urdf::Geometry::SPHERE) {
    auto sphere = std::static_pointer_cast<urdf::Sphere>(shape);
    require(std::isfinite(sphere->radius) && sphere->radius > 0,"sphere 반지름이 잘못됐습니다.");
    return std::make_shared<fcl::Sphered>(sphere->radius);
  }
  if (shape->type == urdf::Geometry::CYLINDER) {
    auto cylinder = std::static_pointer_cast<urdf::Cylinder>(shape);
    require(std::isfinite(cylinder->radius) && std::isfinite(cylinder->length) &&
      cylinder->radius > 0 && cylinder->length > 0,"cylinder 크기가 잘못됐습니다.");
    return std::make_shared<fcl::Cylinderd>(cylinder->radius,cylinder->length);
  }
  require(shape->type == urdf::Geometry::MESH,"지원하지 않는 충돌 형상입니다.");
  auto mesh = std::static_pointer_cast<urdf::Mesh>(shape);
  const Eigen::Vector3d scale(mesh->scale.x,mesh->scale.y,mesh->scale.z);
  require(scale.allFinite() && (scale.array()>0).all(),"메시 scale이 잘못됐습니다.");
  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(mesh_path(mesh->filename),
    aiProcess_Triangulate|aiProcess_JoinIdenticalVertices|aiProcess_PreTransformVertices);
  require(scene && scene->HasMeshes(),"충돌 메시 읽기 실패: "+mesh->filename);
  std::vector<fcl::Vector3d> vertices;
  std::vector<fcl::Triangle> triangles;
  for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
    const auto* part = scene->mMeshes[i];
    const int offset = static_cast<int>(vertices.size());
    for (unsigned int j = 0; j < part->mNumVertices; ++j) {
      const auto& v = part->mVertices[j];
      vertices.emplace_back(v.x*scale.x(),v.y*scale.y(),v.z*scale.z());
      require(vertices.back().allFinite(),"메시 정점이 유효하지 않습니다.");
    }
    for (unsigned int j = 0; j < part->mNumFaces; ++j) {
      const auto& face = part->mFaces[j];
      require(face.mNumIndices == 3,"삼각형 충돌 메시가 필요합니다.");
      triangles.emplace_back(offset+face.mIndices[0],offset+face.mIndices[1],offset+face.mIndices[2]);
    }
  }
  require(!vertices.empty() && !triangles.empty(),"충돌 메시가 비어 있습니다.");
  auto result = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
  require(result->beginModel() == 0 && result->addSubModel(vertices,triangles) == 0 && result->endModel() == 0,
    "FCL 충돌 메시 생성 실패");
  return result;
}
bool adjacent(const urdf::Model& model, const std::string& first, const std::string& second) {
  std::map<std::string,int> ancestors;
  auto link = model.getLink(first);
  int distance = 0;
  while (link) {
    ancestors[link->name] = distance;
    if (link->parent_joint && link->parent_joint->type != urdf::Joint::FIXED) ++distance;
    link = link->getParent();
  }
  link = model.getLink(second); distance = 0;
  while (link) {
    const auto it = ancestors.find(link->name);
    if (it != ancestors.end()) return distance+it->second <= 1;
    if (link->parent_joint && link->parent_joint->type != urdf::Joint::FIXED) ++distance;
    link = link->getParent();
  }
  return false;
}
}  // namespace

std::vector<EnvironmentBox> environment_boxes(const YAML::Node& environment) {
  const auto table = environment["table"];
  const Eigen::Vector3d size = vector3(table["size_m"]);
  require((size.array()>0).all(),"책상 크기는 양수여야 합니다.");
  xm::Pose world_table = xm::Pose::Identity();
  world_table.translation() = vector3(table["position_m"]);
  world_table.linear() = rpy(vector3(table["rpy_rad"]));
  const double floor = number(environment["floor_z_m"]);
  std::vector<EnvironmentBox> result;
  auto enabled = [](const YAML::Node& settings) {
    const auto value = settings["enabled"];
    require(value.IsScalar() && value.Tag() != "!" && value.Tag() != "tag:yaml.org,2002:str" &&
      (value.Scalar() == "true" || value.Scalar() == "false"),"환경 enabled는 bool이어야 합니다.");
    return value.as<bool>();
  };
  if (enabled(table)) {
    xm::Pose center = xm::Pose::Identity(); center.translation() = Eigen::Vector3d(size.x()/2,0,-size.z()/2);
    result.push_back({"environment_table",world_table*center,size});
  }
  for (const auto& name : {"blind","right_wall"}) {
    const auto settings = environment[name];
    if (!enabled(settings)) continue;
    const bool right = std::string(name) == "right_wall";
    const double width = number(settings[right?"length_m":"width_m"]);
    const double height = number(settings["height_m"]), thickness = number(settings["thickness_m"]);
    const double gap = number(settings["gap_from_table_m"]);
    require(width > 0 && height > 0 && thickness > 0 && gap >= 0,"벽 크기·간격이 잘못됐습니다.");
    const Eigen::Vector3d anchor = right ? Eigen::Vector3d(size.x()/2,-size.y()/2-gap,0) : Eigen::Vector3d(size.x()+gap,0,0);
    xm::Pose wall = xm::Pose::Identity();
    wall.translation() = world_table*anchor;
    wall.translation().z() = floor;
    wall.translation() += world_table.linear()*vector3(settings["offset_m"]);
    wall.linear() = world_table.linear()*Eigen::AngleAxisd(right?-std::acos(-1.0)/2:0,Eigen::Vector3d::UnitZ()).toRotationMatrix()*rpy(vector3(settings["rpy_rad"]));
    xm::Pose center = xm::Pose::Identity(); center.translation() = Eigen::Vector3d(thickness/2,0,height/2);
    result.push_back({"environment_"+std::string(name),wall*center,{thickness,width,height}});
  }
  return result;
}

struct CollisionChecker::Impl {
  struct Object {
    std::string name, link;
    int arm = -1;
    size_t frame_index = 0;
    xm::Pose local = xm::Pose::Identity();
    std::unique_ptr<fcl::CollisionObjectd> collision;
    xm::Joints radii = xm::Joints::Zero();
    std::array<bool,7> ancestors{};
  };
  struct FrameNode { KDL::Segment segment; int parent = -1, joint = -1; };
  struct Pair {
    size_t first, second;
    std::array<xm::Joints,2> radii{xm::Joints::Zero(),xm::Joints::Zero()};
  };
  struct Timer {
    double& seconds;
    double started = xm::steady_seconds();
    ~Timer() { seconds += xm::steady_seconds()-started; }
  };
  CollisionSettings settings;
  CollisionStats stats;
  std::array<xm::Pose,2> mounts;
  std::array<std::vector<FrameNode>,2> nodes;
  std::array<std::vector<KDL::Frame>,2> frames;
  std::array<std::map<std::string,size_t>,2> link_frames;
  std::vector<Object> objects;
  std::vector<Pair> pairs;

  static int joint_index(const std::string& name) {
    require(name.size()==6 && name.rfind("joint",0)==0 && name[5]>='1' && name[5]<='7',
            "충돌 URDF의 관절 이름이 잘못됐습니다: "+name);
    return name[5]-'1';
  }
  void update(const DualJoints& joints) {
    Timer timer{stats.fk_seconds};
    for (const auto& q : joints) require(q.allFinite(),"충돌 검사 관절각이 유효하지 않습니다.");
    // Parent-first traversal computes each link once, including links without
    // collision geometry. Multiple collision shapes reuse the same frame.
    for(size_t side=0;side<2;++side) {
      for(size_t i=0;i<nodes[side].size();++i) {
        const auto& node=nodes[side][i];
        frames[side][i]=node.parent<0 ? KDL::Frame::Identity() :
          frames[side][node.parent]*node.segment.pose(node.joint<0 ? 0 : joints[side][node.joint]);
        ++stats.fk_evaluations;
      }
    }
    for (auto& object : objects) {
      if (object.arm < 0) continue;
      object.collision->setTransform(mounts[object.arm]*pose(frames[object.arm][object.frame_index])*object.local);
      object.collision->computeAABB();
      require(finite_aabb(object.collision->getAABB()),"충돌 형상의 좌표 범위를 벗어났습니다.");
    }
  }
  bool inspect(const DualJoints& joints, const std::array<xm::Joints,2>& speeds, double half_interval) {
    require(++stats.samples <= settings.maximum_checks,"충돌 검사 횟수 상한을 초과했습니다.");
    update(joints);
    bool certified = true;
    for (const auto& pair : pairs) {
      ++stats.pair_checks;
      auto& first = objects[pair.first]; auto& second = objects[pair.second];
      const double motion=half_interval*(pair.radii[0].dot(speeds[0])+pair.radii[1].dot(speeds[1]));
      const double threshold = settings.margin+motion+1e-12;
      const auto& a = first.collision->getAABB(); const auto& b = second.collision->getAABB();
      Eigen::Vector3d gap;
      for (int axis = 0; axis < 3; ++axis)
        gap[axis] = std::max({0.,a.min_[axis]-b.max_[axis],b.min_[axis]-a.max_[axis]});
      if (gap.norm() > threshold) { ++stats.aabb_rejections; continue; }
      Timer timer{stats.narrowphase_seconds};
      fcl::CollisionResultd contacts;
      ++stats.collision_queries;
      fcl::collide(first.collision.get(),second.collision.get(),fcl::CollisionRequestd(),contacts);
      fcl::DistanceResultd distance_result;
      double distance=0;
      if(!contacts.isCollision()) {
        ++stats.distance_queries;
        distance=fcl::distance(first.collision.get(),second.collision.get(),fcl::DistanceRequestd(),distance_result);
      }
      require(std::isfinite(distance) && distance >= 0,"충돌 거리 계산 실패: "+first.name+" / "+second.name);
      if (distance <= settings.margin) {
        std::ostringstream message;
        message << "충돌 여유거리 위반: " << first.name << " / " << second.name
          << " 거리=" << distance*1000 << "mm, 요구=" << settings.margin*1000 << "mm";
        throw std::runtime_error(message.str());
      }
      if (distance <= threshold) certified = false;
    }
    return certified;
  }
  void interval(const std::function<DualJoints(double)>& sample, double start, double end,
                const std::array<xm::Joints,2>& speeds, const std::function<bool()>& cancelled, size_t depth=0) {
    require(!cancelled(),"충돌 검사가 취소됐습니다.");
    stats.max_depth=std::max(stats.max_depth,depth);
    const double middle = (start+end)/2;
    if (inspect(sample(middle),speeds,(end-start)/2)) return;
    require(end-start > settings.minimum_interval,"구간 사이 충돌 여유거리를 보증하지 못했습니다.");
    ++stats.subdivisions;
    interval(sample,start,middle,speeds,cancelled,depth+1);
    interval(sample,middle,end,speeds,cancelled,depth+1);
  }
};

CollisionChecker::CollisionChecker(const std::array<std::string,2>& urdfs,
                                 const std::array<xm::Pose,2>& mounts,
                                 const std::vector<EnvironmentBox>& environment,
                                 CollisionSettings settings) : impl_(std::make_unique<Impl>()) {
  require(std::isfinite(settings.margin) && settings.margin > 0 &&
    std::isfinite(settings.interval) && settings.interval > 0 &&
    std::isfinite(settings.minimum_interval) && settings.minimum_interval > 0 &&
    settings.minimum_interval <= settings.interval && settings.maximum_checks > 0,"충돌 검사 설정이 잘못됐습니다.");
  impl_->settings = settings; impl_->mounts = mounts;
  for (const auto& mount : mounts) valid_pose(mount);
  std::array<urdf::Model,2> models;
  for (int side = 0; side < 2; ++side) {
    require(models[side].initFile(urdfs[side]),"충돌 URDF를 읽지 못했습니다.");
    KDL::Tree tree;
    require(kdl_parser::treeFromUrdfModel(models[side],tree),"충돌 KDL 트리를 만들지 못했습니다.");
    std::function<void(urdf::LinkConstSharedPtr,int)> visit;
    visit=[&](urdf::LinkConstSharedPtr link,int parent) {
      const auto segment=tree.getSegment(link->name);
      require(segment!=tree.getSegments().end(),"충돌 링크 경로가 없습니다.");
      Impl::FrameNode node;
      node.segment=segment->second.segment; node.parent=parent;
      if(link->parent_joint && link->parent_joint->type!=urdf::Joint::FIXED) {
        require(link->parent_joint->type==urdf::Joint::REVOLUTE || link->parent_joint->type==urdf::Joint::CONTINUOUS,
                "충돌 모델에는 회전·고정 관절만 사용할 수 있습니다.");
        node.joint=Impl::joint_index(link->parent_joint->name);
      }
      const int index=static_cast<int>(impl_->nodes[side].size());
      impl_->nodes[side].push_back(std::move(node));
      impl_->link_frames[side][link->name]=index;
      for(const auto& child:link->child_links) visit(child,index);
    };
    visit(models[side].getRoot(),-1);
    impl_->frames[side].resize(impl_->nodes[side].size());
    size_t count = 0;
    for (const auto& [name,link] : models[side].links_) {
      for (const auto& collision : link->collision_array) {
        Impl::Object object;
        object.arm = side; object.link = name; object.frame_index=impl_->link_frames[side].at(name);
        object.name = std::string(side==0?"left/":"right/")+name;
        object.local = pose(collision->origin);
        valid_pose(object.local);
        auto shape = geometry(collision->geometry);
        object.collision = std::make_unique<fcl::CollisionObjectd>(shape);
        shape->computeLocalAABB();
        require(finite_aabb(shape->aabb_local),"충돌 형상의 크기가 유효하지 않습니다.");
        // For ancestor joint j, every shape point is within the shape radius
        // plus downstream URDF translation lengths of that joint's pivot.
        // Exclude j's own parent offset; it moves the pivot, not the lever arm.
        double radius=shape->aabb_local.center().norm()+shape->aabb_local.radius()+object.local.translation().norm();
        urdf::LinkConstSharedPtr ancestor=link;
        while(ancestor->parent_joint) {
          require(std::isfinite(radius) && radius>=0,"충돌 형상의 회전 반경이 유효하지 않습니다.");
          const auto& joint=ancestor->parent_joint;
          if(joint->type!=urdf::Joint::FIXED) {
            const int index=Impl::joint_index(joint->name);
            object.radii[index]=radius*(1+1e-12)+1e-12;
            object.ancestors[index]=true;
          }
          const auto& p=joint->parent_to_joint_origin_transform.position;
          const Eigen::Vector3d offset(p.x,p.y,p.z);
          require(offset.allFinite(),"충돌 관절 원점이 유효하지 않습니다.");
          radius+=offset.norm();
          ancestor=ancestor->getParent();
        }
        impl_->objects.push_back(std::move(object)); ++count;
      }
    }
    require(count > 0,"팔 URDF에 충돌 형상이 없습니다.");
  }
  for (const auto& box : environment) {
    require(box.size.allFinite() && (box.size.array()>0).all(),"환경 충돌 크기가 잘못됐습니다.");
    valid_pose(box.world_T_box);
    Impl::Object object; object.name = box.name;
    object.collision = std::make_unique<fcl::CollisionObjectd>(std::make_shared<fcl::Boxd>(box.size),box.world_T_box);
    require(finite_aabb(object.collision->getAABB()),"환경 형상의 크기가 유효하지 않습니다.");
    impl_->objects.push_back(std::move(object));
  }
  for (size_t i = 0; i < impl_->objects.size(); ++i) {
    for (size_t j = i+1; j < impl_->objects.size(); ++j) {
      const auto& a = impl_->objects[i]; const auto& b = impl_->objects[j];
      if (a.arm < 0 && b.arm < 0) continue;
      if (a.arm >= 0 && a.arm == b.arm && adjacent(models[a.arm],a.link,b.link)) continue;
      Impl::Pair pair{i,j};
      if(a.arm>=0) pair.radii[a.arm]+=a.radii;
      if(b.arm>=0) pair.radii[b.arm]+=b.radii;
      // Common ancestor rotations rigidly transform both objects together and
      // cannot change their separation. Only unshared joints contribute.
      if(a.arm>=0 && a.arm==b.arm)
        for(int k=0;k<7;++k) if(a.ancestors[k] && b.ancestors[k]) pair.radii[a.arm][k]=0;
      impl_->pairs.push_back(std::move(pair));
    }
  }
}
CollisionChecker::~CollisionChecker() = default;
void CollisionChecker::check(const DualJoints& joints) {
  impl_->stats={};
  Impl::Timer timer{impl_->stats.total_seconds};
  const std::array<xm::Joints,2> speeds{xm::Joints::Zero(),xm::Joints::Zero()};
  impl_->inspect(joints,speeds,0);
}
void CollisionChecker::check_path(const std::function<DualJoints(double)>& sample, double duration,
                                const std::array<double,2>& speeds, const std::function<bool()>& cancelled) {
  for(double speed:speeds) require(std::isfinite(speed) && speed>=0,"관절 속도 상한이 잘못됐습니다.");
  check_path_joint_bounds(sample,duration,{xm::Joints::Constant(speeds[0]),xm::Joints::Constant(speeds[1])},cancelled);
}
void CollisionChecker::check_path_joint_bounds(const std::function<DualJoints(double)>& sample, double duration,
                                const std::array<xm::Joints,2>& speeds, const std::function<bool()>& cancelled) {
  require(std::isfinite(duration) && duration > 0 && duration <= 600,"경로 검사 시간 범위가 잘못됐습니다.");
  for(const auto& speed:speeds) require(speed.allFinite() && (speed.array()>=0).all(),"관절 속도 상한이 잘못됐습니다.");
  impl_->stats={};
  Impl::Timer timer{impl_->stats.total_seconds};
  const std::array<xm::Joints,2> stopped{xm::Joints::Zero(),xm::Joints::Zero()};
  impl_->inspect(sample(0),stopped,0); impl_->inspect(sample(duration),stopped,0);
  for (double start = 0; start < duration; start += impl_->settings.interval)
    impl_->interval(sample,start,std::min(start+impl_->settings.interval,duration),speeds,cancelled);
}
const CollisionStats& CollisionChecker::stats() const { return impl_->stats; }
std::string CollisionChecker::diagnostics() const {
  const auto& s=impl_->stats;
  std::ostringstream out; out<<std::fixed<<std::setprecision(6)
    <<"total_s="<<s.total_seconds<<" samples="<<s.samples<<" fk_evaluations="<<s.fk_evaluations
    <<" pair_checks="<<s.pair_checks<<" aabb_rejections="<<s.aabb_rejections
    <<" collision_queries="<<s.collision_queries<<" distance_queries="<<s.distance_queries
    <<" subdivisions="<<s.subdivisions<<" max_depth="<<s.max_depth
    <<" fk_transform_s="<<s.fk_seconds<<" narrowphase_s="<<s.narrowphase_seconds;
  return out.str();
}
}  // namespace auto_align
