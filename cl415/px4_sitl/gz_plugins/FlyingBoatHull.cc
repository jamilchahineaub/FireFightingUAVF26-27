// flying boat hull: hydrostatics and hydrodynamics for a hull made of boxes, on a model that leaves the water.
//
// why not gz's own usv plugins: Buoyancy's box clipping (gz-math Box::VolumeBelow) is wrong whenever the water
// plane misses the box centre in the installed gz-math 7.5.2 (0.0103 vs the exact 0.0088 m^3 for a 0.44x0.21x0.29
// box 5 cm off centre), and Hydrodynamics assumes a fully submerged body, so it keeps dragging the aircraft after
// it is airborne. this plugin clips every box exactly against the water plane (6 tetrahedra per box), so the
// submerged volume, its centroid, the wetted part of every face and the waterplane are exact, and every force
// goes to zero when the hull leaves the water.
//
// per element (a box in the link frame):
//   buoyancy       rho g V_sub at the centroid of the submerged part
//   plate force    on the wetted part of the bottom, the sides and the top:
//                  N = 0.5 rho v^2 A sin(tau) (k cos(tau) + cd_bluff sin(tau)), pushing into the hull, only while the
//                  face moves into the water (tau > 0). at small tau on the bottom this is planing lift (k_bottom
//                  ~0.4 for a 20 deg deadrise hull, from Savitsky), at tau = 90 deg it is bluff-body drag (slamming,
//                  sideways drift). the x faces are skipped: between abutting boxes they are internal, and the real
//                  hull is smooth there
//   friction       0.5 rho cf A |v_t| v_t on every wetted face
//   form drag      per group of boxes (the hull, each float): 0.5 rho cd_form A_front |v_h| v_h at the group's
//                  submerged centroid, A_front = the largest submerged cross-section of the group. this is the bow
//                  wave and pressure drag; cd_form sets the hump resistance
//   heave damping  linear, 2 zeta sqrt(rho g A_wp m) shared by waterplane area, on the vertical velocity of each
//                  element's waterplane centroid, so the static float settles (the quadratic terms vanish at rest)
//
//   <plugin filename="libCl415FlyingBoatHull.so" name="cl415::FlyingBoatHull">
//     <link_name>base_link</link_name>
//     <water_level>0</water_level>
//     <water_density>998</water_density>
//     <friction_cf>0.005</friction_cf>
//     <side_k>1.0</side_k>
//     <bluff_cd>1.0</bluff_cd>
//     <heave_damping_ratio>0.6</heave_damping_ratio>
//     <form_drag><hull>0.1</hull><left_float>0.15</left_float></form_drag>
//     <state_topic>/cl415/hull</state_topic>
//     <publish_rate>50</publish_rate>
//     <wake_speed_min>3</wake_speed_min>   <wake_speed_max>6</wake_speed_max>
//     <element><name>fore</name><group>hull</group><pose>x y z 0 0 0</pose><size>lx ly lz</size><k_bottom>0.4</k_bottom>
//              <wake_depth>0</wake_depth></element>
//   </plugin>
//
// wake_depth: elements behind the step run in the forebody's wake hollow. their water level is lowered by
// wake_depth, blended in from wake_speed_min to wake_speed_max of horizontal speed, so the afterbody is dry once
// the hull planes and touches again only above the sternpost angle (the real trim limiter). the same blend
// switches off the plate force on the side and top faces: on the step the flow separates at the chines and the
// sides run dry, and a wetted box side ahead of the cg would otherwise act as a fin on the nose (the model
// water-looped at 8 m/s before this). <chine_separation>false</chine_separation> keeps the sides active.
//
// state topic (gz.msgs.Double_V): t, V_sub [m^3], buoyancy [N], resistance along the horizontal velocity [N],
// vertical plate (planing) force [N], heave damping [N], keel depth below the water [m, + = below], horizontal
// speed [m/s], waterplane area [m^2], vertical speed [m/s], then per element: V_sub, vertical plate force,
// horizontal drag

#include <gz/msgs/double_v.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace cl415
{
using V3 = gz::math::Vector3d;

// ---------------------------------------------------------------- exact box / water-plane geometry
static inline V3 isect(const V3 &_p, const V3 &_q, double _level)
{
  const double t = (_level - _p.Z()) / (_q.Z() - _p.Z());
  return _p + (_q - _p) * t;
}

static inline void addTet(const V3 &_a, const V3 &_b, const V3 &_c, const V3 &_d, double &_vol, V3 &_mom)
{
  const double v = std::fabs((_b - _a).Dot((_c - _a).Cross(_d - _a))) / 6.0;
  _vol += v;
  _mom += (_a + _b + _c + _d) * (0.25 * v);
}

// part of a tetrahedron below z = level: volume and first moment
static void clipTet(const V3 _t[4], double _level, double &_vol, V3 &_mom)
{
  int below[4], above[4], nb = 0, na = 0;
  for (int i = 0; i < 4; i++)
    (_t[i].Z() < _level ? below[nb++] : above[na++]) = i;
  if (nb == 0)
    return;
  if (nb == 4)
  {
    addTet(_t[0], _t[1], _t[2], _t[3], _vol, _mom);
    return;
  }
  if (nb == 1)
  {
    const V3 &a = _t[below[0]];
    addTet(a, isect(a, _t[above[0]], _level), isect(a, _t[above[1]], _level), isect(a, _t[above[2]], _level), _vol, _mom);
    return;
  }
  if (nb == 3)
  {
    const V3 &a = _t[above[0]];
    double vc = 0, vt = 0;
    V3 mc = V3::Zero, mt = V3::Zero;
    addTet(a, isect(a, _t[below[0]], _level), isect(a, _t[below[1]], _level), isect(a, _t[below[2]], _level), vc, mc);
    addTet(_t[0], _t[1], _t[2], _t[3], vt, mt);
    _vol += vt - vc;
    _mom += mt - mc;
    return;
  }
  // two below: a wedge, split into three tetrahedra
  const V3 &a = _t[below[0]], &b = _t[below[1]], &c = _t[above[0]], &d = _t[above[1]];
  const V3 pac = isect(a, c, _level), pad = isect(a, d, _level), pbc = isect(b, c, _level), pbd = isect(b, d, _level);
  addTet(a, pac, pad, b, _vol, _mom);
  addTet(pac, pad, b, pbc, _vol, _mom);
  addTet(pad, b, pbc, pbd, _vol, _mom);
}

// part of a convex polygon below z = level
static std::vector<V3> clipPoly(const std::vector<V3> &_in, double _level)
{
  std::vector<V3> out;
  const size_t n = _in.size();
  for (size_t i = 0; i < n; i++)
  {
    const V3 &p = _in[i], &q = _in[(i + 1) % n];
    const bool pin = p.Z() < _level, qin = q.Z() < _level;
    if (pin)
      out.push_back(p);
    if (pin != qin)
      out.push_back(isect(p, q, _level));
  }
  return out;
}

// area and centroid of a planar convex polygon (fan from the first vertex)
static double polyArea(const std::vector<V3> &_poly, V3 &_c)
{
  _c = V3::Zero;
  if (_poly.size() < 3)
    return 0.0;
  double area = 0.0;
  for (size_t i = 1; i + 1 < _poly.size(); i++)
  {
    const double a = 0.5 * (_poly[i] - _poly[0]).Cross(_poly[i + 1] - _poly[0]).Length();
    area += a;
    _c += (_poly[0] + _poly[i] + _poly[i + 1]) * (a / 3.0);
  }
  if (area > 0)
    _c /= area;
  return area;
}

struct Element
{
  std::string name, group;
  gz::math::Pose3d pose;
  V3 size;
  double kBottom{0.4}, wakeDepth{0.0};
  // per step
  double vol{0}, awp{0}, lwet{0}, keel{0}, plateZ{0}, drag{0}, level{0};
  V3 cvol, cwp;
  bool wet{false};
};

class FlyingBoatHull : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    auto sdf = _sdf->Clone();
    this->linkName = sdf->Get<std::string>("link_name", "base_link").first;
    this->level = sdf->Get<double>("water_level", 0.0).first;
    this->rho = sdf->Get<double>("water_density", 998.0).first;
    this->cf = sdf->Get<double>("friction_cf", 0.005).first;
    this->sideK = sdf->Get<double>("side_k", 1.0).first;
    this->bluffCd = sdf->Get<double>("bluff_cd", 1.0).first;
    this->zeta = sdf->Get<double>("heave_damping_ratio", 0.6).first;
    this->topic = sdf->Get<std::string>("state_topic", "/hull").first;
    this->pubPeriod = 1.0 / sdf->Get<double>("publish_rate", 50.0).first;
    this->wakeV0 = sdf->Get<double>("wake_speed_min", 3.0).first;
    this->wakeV1 = sdf->Get<double>("wake_speed_max", 6.0).first;
    this->chineSep = sdf->Get<bool>("chine_separation", true).first;
    if (sdf->HasElement("form_drag"))
    {
      auto fd = sdf->GetElement("form_drag");
      for (auto e = fd->GetFirstElement(); e; e = e->GetNextElement(""))
        this->cdForm[e->GetName()] = e->Get<double>();
    }
    if (sdf->HasElement("element"))
    {
      for (auto e = sdf->GetElement("element"); e; e = e->GetNextElement("element"))
      {
        Element el;
        el.name = e->Get<std::string>("name", "box").first;
        el.group = e->Get<std::string>("group", "hull").first;
        el.pose = e->Get<gz::math::Pose3d>("pose", gz::math::Pose3d::Zero).first;
        el.size = e->Get<V3>("size", V3(0.1, 0.1, 0.1)).first;
        el.kBottom = e->Get<double>("k_bottom", 0.4).first;
        el.wakeDepth = e->Get<double>("wake_depth", 0.0).first;
        this->elements.push_back(el);
      }
    }
    if (this->elements.empty())
    {
      gzerr << "[FlyingBoatHull] no <element> boxes given\n";
      return;
    }
    this->pub = this->node.Advertise<gz::msgs::Double_V>(this->topic);
    this->ok = true;
    gzmsg << "[FlyingBoatHull] " << this->elements.size() << " boxes on " << this->linkName << ", state on "
          << this->topic << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->ok)
      return;
    if (this->link.Entity() == gz::sim::kNullEntity)
    {
      this->link = gz::sim::Link(this->model.LinkByName(_ecm, this->linkName));
      if (this->link.Entity() == gz::sim::kNullEntity)
        return;
      this->link.EnableVelocityChecks(_ecm, true);
      const auto in = _ecm.Component<gz::sim::components::Inertial>(this->link.Entity());
      this->mass = in ? in->Data().MassMatrix().Mass() : 10.0;
    }
    if (_info.paused)
      return;
    const auto poseOpt = this->link.WorldPose(_ecm);
    const auto velOpt = this->link.WorldLinearVelocity(_ecm);
    const auto omgOpt = this->link.WorldAngularVelocity(_ecm);
    if (!poseOpt || !velOpt || !omgOpt)
      return;
    const gz::math::Pose3d &pose = *poseOpt;
    const V3 &vel = *velOpt, &omg = *omgOpt;
    const V3 origin = pose.Pos();
    auto velAt = [&](const V3 &_p) { return vel + omg.Cross(_p - origin); };

    // pass 1: submerged geometry of every box
    V3 vh0 = vel;
    vh0.Z(0.0);
    const double wake = std::clamp((vh0.Length() - this->wakeV0) / std::max(this->wakeV1 - this->wakeV0, 0.1), 0.0, 1.0);
    double awpTot = 0.0, volTot = 0.0, keelMin = 1e9;
    for (auto &el : this->elements)
    {
      V3 corner[8];
      el.keel = 1e9;
      el.plateZ = el.drag = 0.0;
      el.level = this->level - el.wakeDepth * wake;
      const double lvl = el.level;
      bool any = false;
      for (int i = 0; i < 8; i++)
      {
        const V3 c((i & 1) ? el.size.X() / 2 : -el.size.X() / 2, (i & 2) ? el.size.Y() / 2 : -el.size.Y() / 2,
                   (i & 4) ? el.size.Z() / 2 : -el.size.Z() / 2);
        corner[i] = pose.CoordPositionAdd(el.pose.CoordPositionAdd(c));
        el.keel = std::min(el.keel, corner[i].Z() - this->level);
        any = any || corner[i].Z() < lvl;
      }
      keelMin = std::min(keelMin, el.keel);
      el.wet = any;
      el.vol = el.awp = el.lwet = 0.0;
      if (!any)
        continue;
      // volume: the six tetrahedra around the diagonal 0-7
      static const int tets[6][4] = {{0, 1, 3, 7}, {0, 1, 5, 7}, {0, 2, 3, 7}, {0, 2, 6, 7}, {0, 4, 5, 7}, {0, 4, 6, 7}};
      double vol = 0.0;
      V3 mom = V3::Zero;
      for (const auto &t : tets)
      {
        const V3 tv[4] = {corner[t[0]], corner[t[1]], corner[t[2]], corner[t[3]]};
        clipTet(tv, lvl, vol, mom);
      }
      el.vol = vol;
      el.cvol = vol > 1e-12 ? mom / vol : origin;
      // waterplane: plane crossings of the twelve edges, a convex polygon
      std::vector<V3> cut;
      for (int i = 0; i < 8; i++)
        for (int k = 0; k < 3; k++)
        {
          const int j = i | (1 << k);
          if (j == i)
            continue;
          const bool bi = corner[i].Z() < lvl, bj = corner[j].Z() < lvl;
          if (bi != bj)
            cut.push_back(isect(corner[i], corner[j], lvl));
        }
      if (cut.size() >= 3)
      {
        V3 cc = V3::Zero;
        for (const auto &p : cut)
          cc += p;
        cc /= static_cast<double>(cut.size());
        std::sort(cut.begin(), cut.end(), [&](const V3 &_a, const V3 &_b) {
          return std::atan2(_a.Y() - cc.Y(), _a.X() - cc.X()) < std::atan2(_b.Y() - cc.Y(), _b.X() - cc.X());
        });
        el.awp = polyArea(cut, el.cwp);
      }
      // wetted length along the box x axis (for the frontal-area estimate)
      double xmin = 1e9, xmax = -1e9;
      const V3 ex = pose.Rot().RotateVector(el.pose.Rot().RotateVector(V3::UnitX));
      for (int i = 0; i < 8; i++)
        if (corner[i].Z() < lvl)
        {
          const double x = corner[i].Dot(ex);
          xmin = std::min(xmin, x);
          xmax = std::max(xmax, x);
        }
      for (const auto &p : cut)
      {
        const double x = p.Dot(ex);
        xmin = std::min(xmin, x);
        xmax = std::max(xmax, x);
      }
      el.lwet = std::max(xmax - xmin, 0.02);
      awpTot += el.awp;
      volTot += el.vol;
    }
    if (volTot < 1e-9)
    {
      this->Publish(_info, 0, 0, 0, 0, 0, keelMin, vel, 0);
      return;
    }

    // pass 2: forces
    V3 F = V3::Zero, T = V3::Zero;
    double fBuoy = 0, fPlateZ = 0, fHeave = 0;
    V3 fHydro = V3::Zero;                       // everything but buoyancy, for the resistance readout
    auto apply = [&](const V3 &_f, const V3 &_at) {
      F += _f;
      T += (_at - origin).Cross(_f);
    };
    const double cHeave = awpTot > 1e-6 ? 2.0 * this->zeta * std::sqrt(this->rho * 9.81 * awpTot * this->mass) / awpTot : 0.0;
    struct Group { double vol{0}, afront{0}; V3 mom{V3::Zero}; };
    std::map<std::string, Group> groups;
    for (auto &el : this->elements)
    {
      if (!el.wet || el.vol < 1e-12)
        continue;
      // buoyancy
      const V3 fb(0, 0, this->rho * 9.81 * el.vol);
      apply(fb, el.cvol);
      fBuoy += fb.Z();
      // heave damping on the waterplane centroid
      if (el.awp > 1e-6)
      {
        const double vz = velAt(el.cwp).Z();
        const V3 fh(0, 0, -cHeave * el.awp * vz);
        apply(fh, el.cwp);
        fHeave += fh.Z();
        fHydro += fh;
      }
      // faces: bottom, sides, top (axis 0 = x faces skipped)
      for (int axis = 1; axis < 3; axis++)
        for (int sgn = -1; sgn <= 1; sgn += 2)
        {
          const int b = (axis + 1) % 3, c = (axis + 2) % 3;
          std::vector<V3> quad;
          const int cyc[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
          for (const auto &bc : cyc)
          {
            int idx = (sgn > 0 ? (1 << axis) : 0) | (bc[0] ? (1 << b) : 0) | (bc[1] ? (1 << c) : 0);
            const V3 cv((idx & 1) ? el.size.X() / 2 : -el.size.X() / 2, (idx & 2) ? el.size.Y() / 2 : -el.size.Y() / 2,
                        (idx & 4) ? el.size.Z() / 2 : -el.size.Z() / 2);
            quad.push_back(pose.CoordPositionAdd(el.pose.CoordPositionAdd(cv)));
          }
          const auto wetPoly = clipPoly(quad, el.level);
          V3 cf;
          const double area = polyArea(wetPoly, cf);
          if (area < 1e-6)
            continue;
          V3 nLocal = V3::Zero;
          nLocal[axis] = sgn;
          const V3 n = pose.Rot().RotateVector(el.pose.Rot().RotateVector(nLocal));   // outward
          const V3 v = velAt(cf);
          const double speed = v.Length();
          if (speed < 1e-4)
            continue;
          const double vn = v.Dot(n);                       // + : the face moves into the water
          const V3 vt = v - n * vn;
          const double q = 0.5 * this->rho * speed * speed;
          // friction on the tangential flow
          V3 vhf = v;
          vhf.Z(0.0);
          const V3 dragDir = vhf.Length() > 1e-6 ? -vhf / vhf.Length() : V3::Zero;
          if (vt.Length() > 1e-6)
          {
            const V3 ff = vt * (-0.5 * this->rho * this->cf * area * vt.Length());
            apply(ff, cf);
            fHydro += ff;
            el.drag += ff.Dot(dragDir);
          }
          if (vn > 0)
          {
            const double sinT = vn / speed, cosT = vt.Length() / speed;
            const bool bottom = (axis == 2 && sgn < 0);
            const double k = bottom ? el.kBottom : this->sideK;
            const double cn = sinT * (k * cosT + this->bluffCd * sinT);
            const double sideFactor = (!bottom && this->chineSep) ? 1.0 - wake : 1.0;
            const V3 fn = n * (-q * area * cn * sideFactor);   // into the hull
            apply(fn, cf);
            fHydro += fn;
            fPlateZ += fn.Z();
            el.plateZ += fn.Z();
            el.drag += fn.Dot(dragDir);
          }
        }
      auto &g = groups[el.group];
      g.vol += el.vol;
      g.mom += el.cvol * el.vol;
      g.afront = std::max(g.afront, el.vol / el.lwet);
    }
    // form drag per group
    for (auto &kv : groups)
    {
      auto it = this->cdForm.find(kv.first);
      const double cd = it != this->cdForm.end() ? it->second : 0.1;
      if (kv.second.vol < 1e-12 || cd <= 0)
        continue;
      const V3 cg = kv.second.mom / kv.second.vol;
      V3 vh = velAt(cg);
      vh.Z(0.0);
      const double sh = vh.Length();
      if (sh < 1e-4)
        continue;
      const V3 fd = vh * (-0.5 * this->rho * cd * kv.second.afront * sh);
      apply(fd, cg);
      fHydro += fd;
    }
    F.Correct();
    T.Correct();
    this->link.AddWorldWrench(_ecm, F, T);

    V3 vh = vel;
    vh.Z(0.0);
    const double sh = vh.Length();
    const double resist = sh > 1e-3 ? -fHydro.Dot(vh) / sh : 0.0;
    this->Publish(_info, volTot, fBuoy, resist, fPlateZ, fHeave, keelMin, vel, awpTot);
  }

private:
  void Publish(const gz::sim::UpdateInfo &_info, double _vol, double _fb, double _r, double _fp, double _fh,
               double _keel, const V3 &_vel, double _awp)
  {
    const double t = std::chrono::duration<double>(_info.simTime).count();
    if (t - this->lastPub < this->pubPeriod)
      return;
    this->lastPub = t;
    gz::msgs::Double_V msg;
    for (double x : {t, _vol, _fb, _r, _fp, _fh, -_keel, std::hypot(_vel.X(), _vel.Y()), _awp, _vel.Z()})
      msg.add_data(x);
    for (const auto &el : this->elements)
      for (double x : {el.vol, el.plateZ, el.drag})
        msg.add_data(x);
    this->pub.Publish(msg);
  }

  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Link link{gz::sim::kNullEntity};
  std::string linkName, topic;
  std::vector<Element> elements;
  std::map<std::string, double> cdForm;
  double level{0}, rho{998}, cf{0.005}, sideK{1.0}, bluffCd{1.0}, zeta{0.6}, mass{10.0};
  double pubPeriod{0.02}, lastPub{-1.0}, wakeV0{3.0}, wakeV1{6.0};
  bool chineSep{true};
  gz::transport::Node node;
  gz::transport::Node::Publisher pub;
  bool ok{false};
};
}  // namespace cl415

GZ_ADD_PLUGIN(cl415::FlyingBoatHull, gz::sim::System, cl415::FlyingBoatHull::ISystemConfigure,
              cl415::FlyingBoatHull::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(cl415::FlyingBoatHull, "cl415::FlyingBoatHull")
