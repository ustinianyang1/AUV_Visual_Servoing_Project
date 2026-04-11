# AUV Visual Servoing Project 指南 / Copilot Instructions

## 1. 理论与参数原则
- **严禁修改论文原理**：不允许修改论文原理。
- **参数修改限制**：原则上不允许修改论文参数。
- **例外情况声明**：仅在有明显依据证明参数错误时可以修改，并且必须在此文件中保留记录。

## 2. 图像评估与复现工作流
- **强制比对**：每次生成图像后，都必须与 comparison/Adaptive_Output_Feedback_Trajectory_Tracking_Control_of_an_Indoor_Blimp_Controller_Design_and_Experiment_Validation.pdf-0010-02.png 比对并评估效果。
- **变量合理性检查**：每次运行仿真后，必须检查保存在单独文件夹（如 `data/`）中的主要变量数据是否合理（例如不能包含 `NaN` 或 `Inf`，且数值应在物理限制的合理范围内）。
- **效果评估与排查**：如果效果不佳，需要评估当前项目找出可能的原因，并由我决定修改方向。

---
## 【参数修改记录】
*(如果有任何违反原论文的参数修改，请在此处追加记录)*
