# COMP551805
> 计算机图形学
> 
> Original experimental materials (framework code): [dandelion](https://github.com/XJTU-Graphics/dandelion)

## A. Repository Overview
1. This repository contains all the experimental code that I completed during the Computer Graphics course. Specifically, it includes:
    - All compulsory experiments:
      - Setting up the Dandelion environment (5 points)
      - Transformation matrices: Translation, Rotation, Scaling (5 points)
      - Perspective Projection Matrix (10 points)
    - Euler Angles to Quaternion Conversion
    - **Rendering**:
      - Software Rasterizer: Rasterization Pipeline
      - Software Renderer: Witted Style Ray-Tracing
    - **Physical Simulation**:
      - Solving Rigid Body Motion using Forward Euler Method
      - Solving Rigid Body Motion using Alternative Methods
      - Naïve Collision Detection

   A total of 20 (compulsory) + 51 (optional) = 71 points have been earned.

2. For experiment submissions, attachments are required to include:
   - **src.zip**: Contains the modified source code, with a size less than 500 KiB.
   - **modifications.cpp**: Copies of the complete function definitions that were modified.
   - **screenshots.zip**: Screenshots of the Dandelion window during the experiment.

   Therefore, in addition to the source code, the repository includes a Handout (or possibly Handin, who knows?) folder that contains the submission content (*This also means you could directly read the corresponding modifications.cpp in this folder to quickly get the information you are looking for.* XD).

3. The repository is maintained with multiple branches, each corresponding to different experimental modules (e.g., the physical simulation module is in the `phys-sim-part` branch).
   > You might also notice the existence of a Handin branch, which is due to updates made to the framework code [dandelion](https://github.com/XJTU-Graphics/dandelion) during the experiments. Although I merged updates in `./dandelion`, `./handout` had already been organized into a `.zip` file, and thus was not merged. The Handin branch was created to organize the submission again after updating some parts of the framework code that caused inconsistencies (**mostly type conversion issues that would cause your submission to be rejected!**).

## B. Computer Graphics Experiments Overview
The Computer Graphics course is offered in the 2024 fall semester at Xi'an Jiaotong University, for third-year undergraduate students. The course consists of compulsory and optional experiments. The compulsory experiments are:

1. Setting up the Dandelion Environment (5 points)
2. Transformation Matrices: Translation, Rotation, Scaling (5 points)
3. Perspective Projection Matrix (10 points)

The optional experiments are described as follows:

### 1. Experiment Content and Selection Method
Optional experiments are divided into three major fields—Rendering, Geometry Processing, and Physical Simulation—each covering experiments that correspond to foundational knowledge in graphics related to rendering, geometry, and animation.

- **Rendering**: Covers the fundamentals of modern rendering pipelines. Experiments include Rasterization Pipeline and Ray-Tracing (such as Witted-style ray tracing) to produce realistic reflections and shadow effects.
- **Geometry Processing**: Involves the half-edge data structure and its application to mesh operations, such as local manipulation, remeshing, and surface simplification, enhancing geometric modeling precision.
- **Physical Simulation**: Focuses on approximating movement and collision processes, including the implementation of basic animation physical simulation algorithms and accelerated collision detection.

Some experiments have dependencies—meaning certain prerequisite experiments must be completed before proceeding. All experiments and their dependencies are illustrated as follows:
![XJTU-CG-Experiment Flowchart](https://github.com/rouge3877/ImageHosting/blob/main/XJTU-CG-Flow.png)

### 2. Experiment Evaluation
The maximum score for optional experiments is 70 points, which is combined with the scores from compulsory experiments to calculate the final grade. Even if the total score exceeds 70 points, only 70 points will be counted. To obtain ≥65 points, you must meet at least one of the following conditions:
- Complete at least one challenge task
- Complete at least two experiments from each of two different areas

After completing the experiments, submissions must be emailed for evaluation by teaching assistants. During evaluation, you will need to demonstrate your code, answer questions, and present the program in action. The experimental scores are finalized after the last evaluation and cannot be modified afterward.
