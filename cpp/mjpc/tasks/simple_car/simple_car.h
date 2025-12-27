#ifndef MJPC_TASKS_VEHICLE_TASK_H_
#define MJPC_TASKS_VEHICLE_TASK_H_

#include <memory>
#include <string>

#include <mujoco/mujoco.h>
#include "mjpc/task.h"

namespace mjpc {

class VehicleTask final : public Task {
 public:
  VehicleTask();

  std::string Name() const override;
  std::string XmlPath() const override;

  void TransitionLocked(mjModel* model, mjData* data) override;
  void ModifyScene(const mjModel* model,
                   const mjData* data,
                   mjvScene* scene) const override;

 protected:
  std::unique_ptr<BaseResidualFn> ResidualLocked() const override;
  BaseResidualFn* InternalResidual() override;

 private:
  // 残差缓存
  mutable std::unique_ptr<BaseResidualFn> residual_cache_;

  // 状态变量
  float fuel_pct_ = 100.0f;

  // HUD 工具函数
  static void DrawRing(const float center[3], float radius, mjvScene* scene);
  static void DrawNeedle(const float center[3], float angle_deg, mjvScene* scene);
};

}  // namespace mjpc

#endif  // MJPC_TASKS_VEHICLE_TASK_H_
