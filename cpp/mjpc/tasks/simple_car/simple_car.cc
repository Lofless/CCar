#include "vehicle_task.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>

namespace mjpc {

namespace {

class VehicleResidual final : public BaseResidualFn {
 public:
  explicit VehicleResidual(const VehicleTask* owner)
      : BaseResidualFn(owner) {}

  void Residual(const mjModel* model, const mjData* data,
                double* residual) const override {
    // 位置误差
    residual[0] = data->qpos[0] - data->mocap_pos[0];
    residual[1] = data->qpos[1] - data->mocap_pos[1];

    // 控制误差
    residual[2] = data->ctrl[0];
    residual[3] = data->ctrl[1];
  }
};

inline double Distance2D(double x, double y) {
  return std::sqrt(x * x + y * y);
}

}  // namespace

// ================= VehicleTask =================

VehicleTask::VehicleTask() {
  visualize = 1;
}

std::string VehicleTask::Name() const {
  return "VehicleTask";
}

std::string VehicleTask::XmlPath() const {
  return GetModelPath("vehicle_task/task.xml");
}

std::unique_ptr<BaseResidualFn> VehicleTask::ResidualLocked() const {
  return std::make_unique<VehicleResidual>(this);
}

BaseResidualFn* VehicleTask::InternalResidual() {
  if (!residual_cache_) {
    residual_cache_ = std::make_unique<VehicleResidual>(this);
  }
  return residual_cache_.get();
}

void VehicleTask::TransitionLocked(mjModel* model, mjData* data) {
  double dx = data->mocap_pos[0] - data->qpos[0];
  double dy = data->mocap_pos[1] - data->qpos[1];

  if (Distance2D(dx, dy) < 0.25) {
    static std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<double> uni(-2.0, 2.0);

    data->mocap_pos[0] = uni(rng);
    data->mocap_pos[1] = uni(rng);
    data->mocap_pos[2] = 0.01;
  }

  double speed = Distance2D(data->qvel[0], data->qvel[1]);
  double burn = (0.05 * std::abs(data->ctrl[0]) +
                 0.01 * speed * speed) * model->opt.timestep * 100.0;

  fuel_pct_ = std::max(0.0f, fuel_pct_ - static_cast<float>(burn));
}

void VehicleTask::ModifyScene(const mjModel* model,
                              const mjData* data,
                              mjvScene* scene) const {
  int body_id = mj_name2id(model, mjOBJ_BODY, "car");
  if (body_id < 0) return;

  const double* p = data->xpos + 3 * body_id;
  float center[3] = {
      static_cast<float>(p[0]),
      static_cast<float>(p[1]),
      static_cast<float>(p[2] + 0.35)
  };

  double speed = Distance2D(data->qvel[0], data->qvel[1]) * 3.6;
  double ratio = std::min(speed / 12.0, 1.0);
  float angle = static_cast<float>(180.0 - 180.0 * ratio);

  DrawRing(center, 0.16f, scene);
  DrawNeedle(center, angle, scene);

  if (scene->ngeom < scene->maxgeom) {
    mjvGeom* g = scene->geoms + scene->ngeom++;
    g->type = mjGEOM_LABEL;
    g->pos[0] = center[0];
    g->pos[1] = center[1];
    g->pos[2] = center[2] - 0.06f;
    g->size[0] = g->size[1] = g->size[2] = 0.05f;
    std::snprintf(g->label, sizeof(g->label), "Fuel %d%%",
                  static_cast<int>(fuel_pct_));
    g->rgba[0] = g->rgba[1] = g->rgba[2] = 0.9f;
    g->rgba[3] = 1.0f;
  }
}

// ===== HUD 绘制工具 =====
void VehicleTask::DrawRing(const float c[3], float r, mjvScene* s) {
  if (s->ngeom >= s->maxgeom) return;
  mjvGeom* g = s->geoms + s->ngeom++;
  g->type = mjGEOM_CYLINDER;
  g->pos[0] = c[0];
  g->pos[1] = c[1];
  g->pos[2] = c[2];
  g->size[0] = r;
  g->size[1] = r;
  g->size[2] = 0.002f;
  g->rgba[0] = g->rgba[1] = g->rgba[2] = 0.7f;
  g->rgba[3] = 0.9f;
  for (int i = 0; i < 9; ++i) g->mat[i] = (i % 4 == 0);
}

void VehicleTask::DrawNeedle(const float c[3], float deg, mjvScene* s) {
  if (s->ngeom >= s->maxgeom) return;
  mjvGeom* g = s->geoms + s->ngeom++;
  g->type = mjGEOM_BOX;
  g->size[0] = 0.005f;
  g->size[1] = 0.12f;
  g->size[2] = 0.003f;

  float rad = deg * static_cast<float>(M_PI) / 180.0f;
  g->pos[0] = c[0];
  g->pos[1] = c[1] - 0.06f * std::cos(rad);
  g->pos[2] = c[2] + 0.06f * std::sin(rad);

  float cs = std::cos(rad - M_PI_2);
  float sn = std::sin(rad - M_PI_2);
  float rot[9] = {cs, -sn, 0, sn, cs, 0, 0, 0, 1};
  std::memcpy(g->mat, rot, sizeof(rot));

  g->rgba[0] = 1.0f;
  g->rgba[1] = 0.1f;
  g->rgba[2] = 0.1f;
  g->rgba[3] = 1.0f;
}

}  // namespace mjpc
