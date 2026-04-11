IEEE TRANSACTIONS ON INDUSTRIAL ELECTRONICS, VOL. 72, NO. 4, APRIL 2025 

3960 

## Adaptive Output Feedback Trajectory Tracking Control of an Indoor Blimp: Controller Design and Experiment Validation 

Jinyang Dong , Hai Yu , Biao Lu , _Member, IEEE_ , Huawang Liu , and Yongchun Fang , _Senior Member, IEEE_ 

_**Abstract**_ **—Due to its safety and the flight duration, the indoor blimp robot is an ideal choice for cruise monitoring and inspection. In this article, the robust trajectory tracking problem for a helium-filled blimp is investigated to enhance its adaptability within indoor environments. Specifically, the echo state network (ESN) is employed to estimate the external disturbances and parameter uncertainties experienced by the blimp. To address the issue of unreliable measured velocities, an ESN-based state observer which uses output feedback signals is designed to estimate velocities. Moreover, a tan-type asymmetric barrier Lyapunov function (BLF) is proposed to cope with the time-varying error constraints. All signals in the closed-loop system are proven to be globally ultimately uniformly bounded through Lyapunov techniques. Finally, a series of hardware experiments are conducted on a self-built indoor blimp platform, which demonstrates the effectiveness of the proposed trajectory tracking control scheme.** 

_**Index Terms**_ **—Barrier Lyapunov function (BLF), echo state network (ESN), indoor blimp, state observer.** 

## I. INTRODUCTION 

**W** ITHmaterial technology, the application of aerial robots hasthe development of mechanical, electronic, and been greatly promoted, including military, agriculture, entertainment, rescue, and transportation [1], [2], [3], [4]. However, the majority of unmanned aerial vehicles (UAVs) such as quadcopters, despite their popularity, still face limitations including 

Manuscript received 14 May 2024; revised 31 July 2024; accepted 19 August 2024. Date of publication 12 September 2024; date of current version 11 March 2025. This work was supported in part by the National Natural Science Foundation of China under Grant 62233011, Grant 62203235, and Grant U22A2050; in part by the Key Technologies R&D Program of Tianjin under Grant 23YFZCSN00060; and in part by the Joint Fund of Guangdong Basic and Applied Basic Research Fund under Grant 2022A1515110046. _(Corresponding author: Yongchun Fang.)_ 

The authors are with the Institute of Robotics and Automatic Information System, College of Artificial Intelligence, Nankai University, Tianjin 300071, China, and also with the Institute of Intelligence Technology and Robotic Systems, Shenzhen Research Institute of Nankai University, Shenzhen 518083, China (e-mail: dongjinyang@mail.nankai.edu.cn; yuhai@mail.nankai.edu.cn; lubiao@nankai.edu.cn; huawangl@nankai. edu.cn; fangyc@nankai.edu.cn). 

This article has supplementary downloadable material available at https://doi.org/10.1109/TIE.2024.3451115, provided by the authors. Digital Object Identifier 10.1109/TIE.2024.3451115 

relatively short flight durations, higher energy consumption, and so on. In contrast, blimps gain increasing attention due to their ability to achieve longer flight durations and enhance the safety of human–robot interaction [5], [6], [7]. As a type of lighter-than-air (LTA) aerial vehicle, blimps offer substantial potential for applications requiring low speed and prolonged airborne operations. For instance, Lu et al. employ blimps for cave exploration and simulate search and rescue missions [8]. Additionally, Hou et al. develop the GT-MAB, which is designed to operate safely near humans [9]. Furthermore, blimps are widely used in environmental monitoring [10], unknown environment exploration [11], [12], entertainment [13], and various other applications. 

Despite considerable attention, the research on the indoor blimp is still at an early stage because of the limited load capacity. Due to its relatively large volume, blimps exhibit complex, nonlinear, and undeniable aerodynamic effects. In the early stages of research, many works overlook the dynamics and only use model-free control methods such as proportion-integrationdifferentiation (PID) or fuzzy logic to achieve trajectory tracking [14], [15], [16]. In addition, several research efforts set PID controllers as a standard baseline and explore the enhancement of control performance through deep reinforcement learning techniques [17], [18]. Other approaches involve using linearized kinematic and dynamic models of blimps to apply linear control strategies, such as linear quadratic regulator (LQR) [19], [20], and model predictive control (MPC) [21]. However, neglecting or simplifying dynamic models may impact the trajectory tracking performance of blimps in practical applications. 

Unlike heavier-than-air (HLA) aircraft, blimps require buoyancy to counteract gravity. This characteristic makes blimps more sensitive to external disturbances, posing challenges for their stable flight. One feasible strategy involves the enhancement of tracking accuracy through the estimation and compensation of disturbances. The neural network-based [22], [23], [24], or fuzzy logic-based [25], [26] controller is an effective method for handling model uncertainties. However, it is worth noting that the calculation of multilayer neural networks and fuzzy logic rules are both time-consuming. In this regard, an echo state network (ESN) is proposed in [27]. Compared with other recursive architectures, ESN is practical, conceptually simple, and easy to implement [28]. Besides, it employs multiple high-dimensional projections in a large number of reservoir 

0278-0046 © 2024 IEEE. Personal use is permitted, but republication/redistribution requires IEEE permission. See https://www.ieee.org/publications/rights/index.html for more information. 

Authorized licensed use limited to: NANKAI UNIVERSITY. Downloaded on February 09,2026 at 05:28:07 UTC from IEEE Xplore.  Restrictions apply. 

DONG et al.: ADAPTIVE OUTPUT FEEDBACK TRAJECTORY TRACKING CONTROL OF INDOOR BLIMP 

3961 

states with strong nonlinear mapping capabilities, to capture the dynamics of the input. In real-world applications, position information is often readily available, whereas velocity information may not be. Therefore, studying the trajectory tracking problem without velocity information is important. To address situations where some states are unmeasurable, the linear observer and high-gain observer are proposed to achieve acceptable results [29], [30], [31]. In addition to uncertainties, constraints are commonly encountered in various dynamic systems. These constraints often arise due to security concerns and physical limitations, leading to different types of state constraints during system operation [32], [33]. Violating these constraints may result in adverse consequences, including system degradation, reduced performance, or even complete instability. Therefore, it is essential to ensure these constraints, so as to make the system stable and reliable [34], [35]. The barrier Lyapunov function (BLF) is an effective method for solving error constraints. It can ensure that the constraints are met and the state remains within the expected range despite the existence of uncertainty [36], [37]. 

This article builds a novel type of indoor blimp robot that can safely operate in close proximity to humans. The blimp has a large volume and obvious inertia characteristics, making it difficult to accurately measure model parameters. During flight, the position and velocity information of the blimp is measured through visual-inertial odometry (VIO). However, the measurements, especially for the velocity information, usually contain noise caused by various reasons. These factors lead to many challenges for the trajectory tracking problem of blimps, making it extremely difficult to ensure tracking accuracy. To handle the above issues, an ESN-based state observer is designed to estimate system uncertainties and velocity information. Furthermore, an asymmetric BLF is utilized to constrain the tracking error within the expected time-varying range. Subsequently, the stability of the closed-loop system is guaranteed by Lyapunov techniques. Finally, the proposed control scheme is validated through convincing comparison experimental tests. The main contributions of this article are as follows. 

- 1) In contrast to the neural network-based or fuzzy logicbased identifiers developed in [22], [23], [24], [25], and [26], the ESN is used to estimate the uncertain dynamics suffered by the blimp, with fewer tuning parameters that can improve convergence speed. Furthermore, unlike the systems in [23], [24], and [25], which can obtain complete and reliable state feedback, the velocity information obtained by blimps is often unreliable. Therefore, the ESN-based state observer is proposed to estimate the velocity of the blimp, thereby addressing the output feedback problem. 

- 2) Different from the approaches of ignoring or linearizing dynamics in [5], [10], [14], [15], [16], [19], and [20], the dynamics of the blimp are fully considered without any linearization. For indoor blimp trajectory tracking, a backstepping controller incorporating the tan-type asymmetric barrier Lyapunov function (TABLF) method is designed, which constructs a virtual velocity signal to deal with the blimp kinematics. The controller manages to constrain the 

**==> picture [144 x 129] intentionally omitted <==**

**==> picture [37 x 224] intentionally omitted <==**

**==> picture [144 x 97] intentionally omitted <==**

Fig. 1. Definition of the inertial and body-fixed coordinate system of the indoor blimp. 

   - tracking error within a specified time-varying range and ensure convergence of the blimp’s position and attitude to the expected values. 

- 3) The experimental results demonstrate that the proposed control method exhibits smaller tracking errors in contrast to the proportional derivative (PD) method with/without the ESN-based observer. Moreover, it proves to be robust even in the presence of various disturbances. 

The remainder of this article is organized in the following manner. Section II covers the preliminaries and the dynamic model of the indoor blimp with multiple-inputs multiple-outputs (MIMOs). The observer and controller design process, as well as the stability analysis, is provided in Section III. In Section IV, several groups of comparison experiments verify the practicability and robustness of the proposed method. Finally, Section V summarizes the article and presents the main conclusion. 

## II. PRELIMINARIES AND PROBLEM FORMULATION 

## _A. Notation_ 

The following notations are employed in this article. R _[n][×][n]_ denotes _n × n_ dimensional Euclidean space. diag( _·_ ) represents a diagonal matrix. _λ_ max( _·_ ) and _λ_ min( _·_ ) denote the maximum and minimum eigenvalue of a square matrix( _·_ ). tr( _·_ ) represents the trace of matrix( _·_ ). _∥·∥_ F and _∥·∥_ are the Frobenius norm and Euclidean norm. 

## _B. Blimps Dynamic Model_ 

As shown in Fig. 1, the general kinematic and dynamic models of a blimp can be described in inertial and body-fixed coordinate systems, where _{x, y, z}_ and _{φ, θ, ψ}_ represent 

Authorized licensed use limited to: NANKAI UNIVERSITY. Downloaded on February 09,2026 at 05:28:07 UTC from IEEE Xplore.  Restrictions apply. 

IEEE TRANSACTIONS ON INDUSTRIAL ELECTRONICS, VOL. 72, NO. 4, APRIL 2025 

3962 

blimp’s position and attitude in inertial frame, _{υx, υy, υz}_ and _{ωx, ωy, ωz}_ represent blimp’s linear and angular velocities in body-fixed frame. Due to the relatively slow flight speed of blimps, the roll angle _φ_ and pitch angle _θ_ are considered negligible during the flight process. _mR_ represents the rigid mass of the blimp, _IRBz_ is the moment of inertia around the _z_ -axis, _xg_ is the blimp center of gravity in body-fixed coordinate. _mAx, mAy_ and _mAz_ are the additional masses. _IAz_ is the additional moment of inertia around the _z_ -axis. _fG_ and _fB_ are the gravity and buoyancy acting on the blimp. The nonlinear mathematical model with disturbances can be expressed as follows: 

**==> picture [202 x 10] intentionally omitted <==**

**==> picture [202 x 11] intentionally omitted <==**

where _**η**_ = [ _x, y, z, ψ_ ] _[⊤] ∈_ R[4] , _**υ**_ = [ _υx, υy, υz, ωz_ ] _[⊤] ∈_ R[4] . _**J**_ ( _**η**_ ) _∈_ R[4] _[×]_[4] is the rotation matrix. _**M** ∈_ R[4] _[×]_[4] represents the inertia matrix, which is a positive definite constant matrix. _**C**_ ( _**υ**_ ) _∈_ R[4] _[×]_[4] is the Coriolis-Centripetal matrix. _**D** ∈_ R[4] _[×]_[4] is the damping matrix, and _**g** ∈_ R[4] is the restore force. _**τ** ∈_ R[4] represents system control input, _**τ** E ∈_ R[4] denotes external disturbance. The specific form of these matrices and vectors are _**J**_ ( _**η**_ ) = [ _Jij_ ] _,_ _**M**_ = [ _mij_ ] _,_ _**C**_ ( _**ν**_ ) = [ _cij_ ] _,_ _**D**_ = [ _dij_ ] _,_ _**G**_ = [ _gi_ ] _, i, j_ = (1 _,_ 2 _,_ 3 _,_ 4), and the elements are zeros except for the following ones: _J_ 11 = _J_ 22 = cos _ψ, J_ 12 = _−J_ 21 = _−_ sin _ψ, J_ 33 = _J_ 44 = 1 _, m_ 11 = _mR_ + _mAx, m_ 22 = _mR_ + _mAy, m_ 24 = _m_ 42 = _mRxg, m_ 33 = _mR_ + _mAz, m_ 44 = _IRBz_ + _IAz.c_ 14 = _−c_ 41 = _−_ ( _mR_ + _mAy_ ) _υy − mRxgωz, c_ 24 = _−c_ 42 = ( _mR_ + _mAx_ ) _υx, d_ 11 = _Dυx , d_ 22 = _Dυy , d_ 33 = _Dυz , d_ 44 = _Dωz , g_ 3 = _fG − fB_ . To facilitate description, _**J**_ is used instead of _**J**_ ( _**η**_ ) in the subsequent analysis. Besides, it is worth noting _⊤_ 4 that _**J**[⊤]_ _**J**_ = _**I**_ 4 _×_ 4 and _**J**_[˙] _**J**_ = _ωz_ _**S**_ 4, where _**S**_ 4 _∈_ R . The terms in _**S**_ 4 are explicitly provided as _S_ 21 = _−S_ 12 = 1, and the remaining elements are all zeros. 

In practical applications, it is difficult to obtain a precise model of blimps. The system uncertainties of the blimp are considered as follows: 

**==> picture [191 x 26] intentionally omitted <==**

where _**M**_ 0 _,_ _**C**_ 0( _**υ**_ ) _,_ _**D**_ 0 _,_ _**g**_ 0( _**η**_ ) are the definite parts with _**M**_ 0 being invertible, **Δ** _**M**_ , **Δ** _**C**_ ( _**υ**_ ) _,_ **Δ** _**D** ,_ **Δ** _**g**_ ( _**η**_ ) represent the uncertain parts. Thus, equation (2) can be rewritten as follows: 

**==> picture [211 x 11] intentionally omitted <==**

where _**τ** D ∈_ R[4] contains external disturbances and parameter uncertainties. In addition, it is further defined as follows: 

**==> picture [200 x 11] intentionally omitted <==**

Thus, the dynamic model (3) can be written as follows: 

**==> picture [183 x 13] intentionally omitted <==**

where _**f**_ ( _χ_ ) = _**C**_ 0( _**ν**_ ) _**ν**_ + _**D**_ 0 _**ν** −_ _**τ** D_ includes unknown and unmeasurable disturbances. 

The control objective of this work is to design an adaptive controller for the blimp, enabling it to track a time-varying desired trajectory even under external disturbances and parameter 

**==> picture [242 x 120] intentionally omitted <==**

Fig. 2. Schematic of the control structure and communication signal transmission (Black solid lines: control signals. Red dotted lines: communication signals). 

uncertainties. In detail, define the tracking error as _**e**_ = _**η** −_ _**η** d_ , which needs to be kept in the time-varying constrained range as follows: 

**==> picture [218 x 11] intentionally omitted <==**

where _kli_ ( _t_ ) and _khi_ ( _t_ ) are the permitted lower and upper bounds of tracking error, with _−kli_ (0) _< ei_ (0) _< khi_ (0). 

**==> picture [84 x 8] intentionally omitted <==**

In this section, a nonlinear control scheme is proposed to ensure that the tracking errors of the blimp are kept within the expected bounds in the presence of external disturbances and dynamic uncertainties. The control structure and communication signal transmission of the system are shown in Fig. 2. Besides, the ESN-based observer is utilized to estimate model uncertainties and unavailable velocities, and the TABLF is used to handle asymmetric time-varying error constraints. All signals in the closed-loop system are proved to be globally ultimately uniformly bounded if the initial errors are within the expected ranges. 

## _A. Output Feedback Observer Design_ 

**==> picture [252 x 57] intentionally omitted <==**

It has been pointed out that ESN has universal approximation capability [38], [39]. Therefore, the following ESN is designed to estimate the disturbances: 

**==> picture [176 x 13] intentionally omitted <==**

where _**W**_ out _∈_ R _[n][×]_[4] is the ideal output weight matrix. **Φ** _∈_ R _[n]_ denotes a reservoir vector and _**ε**_ 1 _∈_ R[4] is the bounded approximation error satisfying _∥_ _**ε**_ 1 _∥≤ ε_ ¯1 with _ε_ ¯1 _∈_ R+. The dynamics of **Φ** is given as follows: 

**==> picture [218 x 11] intentionally omitted <==**

where _σ_ ( _·_ ) represents an activation function, which is chosen as the Gaussian function. There is a positive constant _φ_[¯] _∈_ R+ 

Authorized licensed use limited to: NANKAI UNIVERSITY. Downloaded on February 09,2026 at 05:28:07 UTC from IEEE Xplore.  Restrictions apply. 

DONG et al.: ADAPTIVE OUTPUT FEEDBACK TRAJECTORY TRACKING CONTROL OF INDOOR BLIMP 

3963 

satisfying �� **Φ** ( _**χ**_ )��F _[≤][φ]_[¯][as][the][Gaussian][function][is][bounded.] Similar as [40] and [41], the input vector is selected as _**u**_ = � _**x**[⊤]_ 1[(] _[t]_[)] _[,]_ _**[ x]**[⊤]_ 1[(] _[t][ −][t][d]_[)] _[,]_ _**[ x]**[⊤]_ 1[(] _[t][ −]_[2] _[t][d]_[)] _[,]_ _**[ τ]**[ ⊤]_[�] _[⊤]_[.] _[t][d]_[is][a][positive][time] delay constant. _**W**_ in _∈_ R _[n][×]_[12] and _**W** d ∈_ R _[n][×][n]_ denote the input matrix and the internal matrix. 

_Remark 1:_ When using neural networks to estimate the noise experienced by the system, the noise is usually assumed to be bounded [42], [43], [44], [45], [46]. As for the continuity of noise, it does not significantly impact the performance of controller. Specifically, neural networks can effectively manage continuous noise, allowing for smoother adjustments and maintaining system stability. For discontinuous but bounded noise, neural networks still perform effectively by keeping tracking errors within acceptable limits. Both of these scenarios will be reflected in subsequent experiments. 

Then, based on the model of the indoor blimp (2) and (3), the ESN-based observer is designed to estimate the velocity information and unknown disturbances by using the recorded input data _**τ**_ and the output feedback _**x**_ 1. The specific form of the observer can be described as follows: 

**==> picture [244 x 35] intentionally omitted <==**

where _**x**_ ˆ 1 and _**x**_ ˆ 2 are the estimations of _**x**_ 1 and _**x**_ 2. _**x**_ ˜ 1 = _**x**_ ˆ 1 _−_ _**x**_ 1 and _**x**_ ˜ 2 = _**x**_ ˆ 2 _−_ _**x**_ 2 are the position and velocity estimation errors. _**K**_ 1 _∈_ R[4] + _[×]_[4] is a positive definite gain matrix and _**K**_ 2 _∈_ R[4] + _[×]_[4] is a positive definite, diagonal gain matrix. _**W**_[ˆ] out is the estimation of _**W**_ out. The adaptive update law of _**W**_[ˆ] out is given as follows: 

**==> picture [201 x 19] intentionally omitted <==**

where _ρ ∈_ R+ and _kW ∈_ R+ are positive constants. Subtracting (6) from (9) yields 

**==> picture [237 x 40] intentionally omitted <==**

_⊤_ where _**W**_[˜] out = _**W**_[ˆ] out _−_ _**W**_ out. Define _**X**_ = _**x**_ ˜ _[⊤]_ 1 _[,]_[ ˜] _**[x]**[⊤]_ 2 , the dy� � namics of the estimation errors (11) can be rewritten as follows: 

**==> picture [195 x 16] intentionally omitted <==**

**==> picture [191 x 10] intentionally omitted <==**

where 

**==> picture [142 x 52] intentionally omitted <==**

Since the matrix _**A**_ is time-varying, it presents challenges for direct use in stability analysis. To cope with this problem, a blockdiagonal transformation _**Z**_ = _**T X**_ with _**T**_ = diag([ _**J**[⊤] ,_ _**I**_ 4 _×_ 4]) is devised. Besides, _**X**_ = _**T**[⊤]_ _**Z**_ holds because _**J**[⊤]_ _**J**_ = _**I**_ 4 _×_ 4 

exists. Taking the time derivative of _**Z**_ and then substituting (12) into the result, one can obtain 

**==> picture [230 x 49] intentionally omitted <==**

with 

**==> picture [154 x 25] intentionally omitted <==**

which is a constant matrix. _**S**_ = diag([ _**S**_ 4 _,_ **0** 4 _×_ 4]). The timevarying yaw rate _ωz_ satisfies _|ωz| ≤ ω_ ¯ _z_ with _ω_ ¯ _z ∈_ R+. If _**K**_ 1 = _**K**[′]_ 1[+] _[ω]_[¯] _[z]_ _**[S]**[′]_ 4[, where] _**[ K]**[′]_ 1[is a positive definite, diagonal matrix,] _**S**[′]_ 4 _[∈]_[R][4][ with all terms in] _**[ S]**[′]_ 4[being zeros expect for] _[ S]_ 21 _[′]_[=] _[ S]_ 12 _[′]_[=] 1, then _**A**_ 0 + _ω_ ¯ _z_ _**S**_ will be Hurwitz. In this way, considering the Lyapunov equation, there exist positive definite matrices _**P**_ and _**Q**_ satisfying 

**==> picture [232 x 27] intentionally omitted <==**

where _**E**_ = _**C**[⊤] −_ _**P B**_ . 

_Lemma 1:_ Selecting _kW_ such that _kW > φ_[¯][2] , the observer error signals _**x**_ ˜ 1 _,_ ˜ _**x**_ 2 and the output weight matrix estimation error _**W**_[˜] out are uniformly ultimately bounded. _Proof:_ Construct the following Lyapunov function candidate as follows: 

**==> picture [211 x 21] intentionally omitted <==**

Taking the time derivate of (16) yields 

**==> picture [247 x 77] intentionally omitted <==**

Considering Young’s inequality, the multiplicative terms _⊤_ ˜ _⊤_ _**Z**[⊤]_ _**EW**_[˜] out **[Φ]**[(] _**[χ]**_[)] _[,] −_ _**Z**[⊤]_ _**P Bε**_ 1 and _−_ tr _**W**_ out _**[W]**_[ out] can � � be decomposed into a sum of squares as follows: 

**==> picture [252 x 116] intentionally omitted <==**

Authorized licensed use limited to: NANKAI UNIVERSITY. Downloaded on February 09,2026 at 05:28:07 UTC from IEEE Xplore.  Restrictions apply. 

IEEE TRANSACTIONS ON INDUSTRIAL ELECTRONICS, VOL. 72, NO. 4, APRIL 2025 

3964 

By replacing these multiplicative terms, (17) can be transformed into 

It is clear that _V_ 2 is differentiable and continuous if the initial tracking error satisfies _−kli_ (0) _< z_ 1 _i_ (0) _< khi_ (0), and the error is restrained within _−kli_ ( _t_ ) _< z_ 1 _i_ ( _t_ ) _< khi_ ( _t_ ). Thus, provided that the initial errors are within the expected ranges, the range value of _πz_ 1[2] _i_[(] _[t]_[)] _[/]_[2] _[k] li_[2][(] _[t]_[)][ and] _[ πz]_ 1[2] _i_[(] _[t]_[)] _[/]_[2] _[k] li_[2][(] _[t]_[)][ are both from 0 to] _π/_ 2, indicating that _V_ 2 _≥_ 0 holds. 

**==> picture [240 x 80] intentionally omitted <==**

Calculating the time derivative of _V_ 2 obtains 

**==> picture [251 x 74] intentionally omitted <==**

Substituting (15) into (17) yields 

**==> picture [225 x 63] intentionally omitted <==**

where 

**==> picture [252 x 127] intentionally omitted <==**

where _c_ 1 = min � _λ_ min( _**Q**_ ) _/λ_ max( _**P**_ ) _, ρ_ � _kW − φ_[¯][2][��] _, c_ 2 = ¯ _**ε**_[2] 1 _[/]_ 2 + _kW ∥_ _**W**_ out _∥_[2] F _[/]_[2.][Thus,][the][transformation][state] _**[Z]**_[and][the] output weight matrix estimation error _**W**_[˜] out are bounded. Noticing _∥_ _**T**[⊤] ∥<_ 1 and using _**X**_ = _**T**[⊤]_ _**Z**_ , the estimation error _**X**_ is also bounded. 

## _B. Controller Design_ 

To ensure robust tracking, the control law design process is divided into two layers. First, a kinematics virtual controller based on the TABLF technique is utilized to keep the tracking errors within predefined bounds. Subsequently, leveraging the outcomes of the kinematics controller and estimations from the ESN-based observer, a robust controller is designed to ensure tracking stability by compensating for disturbances. 

**==> picture [224 x 100] intentionally omitted <==**

_Step 1_ Define the tracking error vector _**z**_ 1 = _**e** ∈_ R[4] _[×]_[1] and _**z**_ 2 = _**x**_ 2 _−_ _**υ** c ∈_ R[4] _[×]_[1] , where _**υ** c_ is the virtual control input. Furthermore, subtracting (6) from the time derivative of _**z**_ 1 and _**z**_ 2 yields 

**==> picture [196 x 27] intentionally omitted <==**

where _**J** i_ represents the _i_ th line of _**J**_ . Taking the limited constraints and the desired velocities into account, the kinematics virtual control scheme _**υ** c_ can be designed as follows: 

To ensure that the four components of the dynamic consistency error _**z**_ 1 are not too large when encountering unknown disturbances, the TABLF is introduced as follows: 

**==> picture [167 x 13] intentionally omitted <==**

where **Λ** = [Λ1 _,_ Λ2 _,_ Λ3 _,_ Λ4] _[⊤]_ . In detail 

**==> picture [201 x 59] intentionally omitted <==**

**==> picture [252 x 64] intentionally omitted <==**

with 

**==> picture [198 x 30] intentionally omitted <==**

where _**K**_ 3 _∈_ R[4] + _[×]_[4] is a positive definite, diagonal matrix. Substituting (28) and (29) into (27) yields 

_Remark 2:_ From the formation of TABLF expressed by (21) and (22), it can be seen that 

**==> picture [454 x 61] intentionally omitted <==**

Authorized licensed use limited to: NANKAI UNIVERSITY. Downloaded on February 09,2026 at 05:28:07 UTC from IEEE Xplore.  Restrictions apply. 

DONG et al.: ADAPTIVE OUTPUT FEEDBACK TRAJECTORY TRACKING CONTROL OF INDOOR BLIMP 

3965 

where _**U**_ = _**K**_ 2 _**C**[⊤]_ _**C**_ . Substituting (37) into (36) yields 

_Step 2_ Considering the velocity error _**z**_ 2, the following Lyapunov function candidate is selected as 

**==> picture [436 x 123] intentionally omitted <==**

Substituting (20) and (30) into the time derivate of _V_ 3 yields 

**==> picture [234 x 73] intentionally omitted <==**

with _c_ 3 = min �( _λ_ min( _**Q**_ ) _−λ_ max( _**U**_ )) _/λ_ max( _**P**_ ) _, ρ_ � _kW −φ_[¯][2][�] _, λ_ min( _**K**_ 3 _−_ 2 _**K** e_ ) _,_ (2 _λ_ min( _**K**_ 4) _− λ_ max( _**K**_ 2)) _/ λ_ max( _**M**_ 0) _}_ . Thus, one can obtain that 

To stabilize _**z**_ 2, the control law is designed as 

**==> picture [212 x 13] intentionally omitted <==**

**==> picture [195 x 25] intentionally omitted <==**

where **Ψ** = [Ψ1 _,_ Ψ2 _,_ Ψ3 _,_ Ψ4] _[⊤]_ , _**K**_ 4 _∈_ R[4] + _[×]_[4] is a positive definite, diagonal matrix. However, the proposed control law (33) is not applicable in practice as the function _**f**_ ( _χ_ ) contains unknown _**τ** D_ and the unreliable measured velocity _**x**_ 2. As shown in the introduction, the ESN-based state observer is used to estimate the model uncertainty and external disturbances. Thus, the control law can be rewritten as follows: 

where _V_ (0) is the initial value of _V_ . The above inequation implies that _V_ 2 _i,_ _**Z** ,_ _**W**_[˜] out _,_ ˆ _**z**_ 2 are all bounded. Besides, the bounded _V_ 2 _i_ implies that the tracking error _z_ 1 _i_ satisfies 

**==> picture [253 x 41] intentionally omitted <==**

**==> picture [233 x 15] intentionally omitted <==**

This indicates that _−kli_ ( _t_ ) _< z_ 1 _i_ ( _t_ ) _< khi_ ( _t_ ) _, ∀t >_ 0 will hold if the initial condition _−kli_ (0) _< z_ 1 _i_ (0) _< khi_ (0) _, i_ = 1 _,_ 2 _,_ 3 _,_ 4 is satisfied. Thus, the tracking errors are kept within the asymmetrical time-varying constraints despite unknown dynamics and unreliable velocities information. All signals in the closed-loop system are globally ultimately uniformly bounded. 

where _**z**_ ˆ2 = _**x**_ ˆ 2 _−_ _**ν** c ∈_ R[4] _[×]_[1] . 

## _C. Stability Analysis_ 

_Theorem 1:_ For the proposed controller, if the control gain _**K**_ 2, _**K**_ 3 and _**K**_ 4 are chosen as _k_ 3 _i >_ 2 _kei,_ ( _i_ = 1 _,_ 2 _,_ 3 _,_ 4) and 2 _λ_ min( _**K**_ 4) _> λ_ max( _**K**_ 2), then all signals of the closed-loop system are globally ultimately uniformly bounded. 

_Remark 3:_ During the flight of blimps, model uncertainty and external disturbances are inevitable, and it is always difficult to estimate these disturbances accurately. Through the above theoretical analysis, it is proved that all signals in the closedloop system are ultimately uniformly bounded, which means the system is robust to disturbances. It can adapt to changes within a certain range without becoming unstable. Meanwhile, if the system is completely known and free from disturbances, _**ε**_ 1 = **0** and _**W**_ out = **0** will be satisfied. The inequation (39) can be rewritten as 0 _≤ V ≤ V_ (0) _e[−][c]_[3] _[t]_ , which implies that the controller can make all signals in the closed-loop system eventually converge to zero. 

_Proof:_ Construct the Lyapunov function as follows: 

**==> picture [183 x 21] intentionally omitted <==**

Taking the time derivative of (35), and substituting (9) and (34) into (36) yields 

**==> picture [249 x 119] intentionally omitted <==**

## IV. EXPERIMENTAL IMPLEMENTATION AND RESULTS 

## _A. Experimental Platform_ 

The experiments are carried out on a self-built testbed shown in Fig. 3. The devised indoor blimp platform consists of a helium-filled envelope with a gondola underneath. The diameter of the envelope is 1 _._ 2 m, which offsets the gravity of the platform. The six M1106-KV7500 brushless motors provided propulsion for the blimp with a wingspan of 350 mm. An Esp Wroom 32 control unit is connected to the onboard computer and sends the control signals to change the speed of the motors. 

By using the Young’s inequality, it is shown 

**==> picture [207 x 21] intentionally omitted <==**

Authorized licensed use limited to: NANKAI UNIVERSITY. Downloaded on February 09,2026 at 05:28:07 UTC from IEEE Xplore.  Restrictions apply. 

IEEE TRANSACTIONS ON INDUSTRIAL ELECTRONICS, VOL. 72, NO. 4, APRIL 2025 

3966 

**==> picture [246 x 147] intentionally omitted <==**

Fig. 3. Experimental platform. 

The onboard computer is chosen as Nvidia Jetson Orin NX 16G which runs the Ubuntu 20.04 operating system and connects to the ground station via WIFI with the 5G band. The Intel Realsense t265 is utilized to obtain the position and attitude of the blimp, and then send the signals to the controller at a frequency of 200 Hz. The air blower is used to simulate the external disturbances. The whole system runs with the support of the robot operating system (ROS). The physical parameters of the self-built platform are given as _mR_ = 0 _._ 884 kg _, mAx_ = _mAy_ = 0 _._ 585 kg _, mAz_ = 0 _._ 607 kg _, IRBz_ = 0 _._ 039 kg _·_ m[2] _, IAz_ = 0 _._ 012 kg _·_ m[2] _, xg_ = 0 _._ 05 m _, Dvx_ = _Dvy_ = 0 _._ 031 kg/s _, Dvz_ = 0 _._ 025 kg/s _, Dωz_ = 0 _._ 001 kg _·_ m/s _, g_ = 9 _._ 8 m/s[2] . 

There are eight neurons in the hidden layer. The control period is 100 Hz, which implies the time delay as _td_ = 0 _._ 01 s. Besides, the positions tracking errors _zi,_ ( _i_ = 1 _,_ 2 _,_ 3 _,_ 4) are required not to violate following asymmetric time-varying constraints: 

**==> picture [253 x 61] intentionally omitted <==**

After careful tunning, the control parameters are chosen as _ρ_ = 200 _, kw_ = 0 _._ 01, _**K**[′]_ 1[=][ diag][([][1] _[.]_[2] _[,]_[ 1] _[.]_[2] _[,]_[ 2] _[.]_[0] _[,]_[ 0] _[.]_[8][])] _[,]_ _**[K]**_[2][=] diag([1 _._ 8 _,_ 1 _._ 8 _,_ 3 _._ 0 _,_ 1 _._ 4]) _,_ _**K**_ 3 = diag([0 _._ 5 _,_ 0 _._ 5 _,_ 1 _._ 1 _,_ 0 _._ 5]), _**K**_ 4 = diag([1 _._ 4 _,_ 1 _._ 4 _,_ 2 _._ 6 _,_ 1 _._ 2]). 

Besides, two control schemes are chosen for comparison to demonstrate the proposed method’s tracking stability and disturbance rejection ability. The first scheme is the PD controller, defined by the equation _**τ**_ 1 = _−_ _**K** p_ _**z**_ 1 _−_ _**K** d_ ˙ _**z**_ 1, where _**z**_ ˙ 1 is derived by differentiating _**z**_ 1. The control gains are carefully tuned to yield a satisfactory performance and finally set as _**K** p_ = diag([1 _._ 4 _,_ 1 _._ 4 _,_ 1 _._ 2 _,_ 5]) and _**K** d_ = diag([2 _._ 6 _,_ 2 _._ 6 _,_ 2 _._ 0 _,_ 7 _._ 5]). The second scheme is the PD method combined with the ESN-based observer, whose mathˆ ematical expression is given as _**τ**_ 2 = _−_ _**K** p_ _**z**_ 1 _−_ _**K** d_ _**z**_[˙] 1 + _**W**_ ˆ _⊤_ out **[Φ]**[(] _**[χ]**_ 1[)][ with] _**[z]**_[˙ˆ][1][=] _**[ J]**_[ ˆ] _**[x]**_[2] _[−]_ _**[x]**_[˙][1] _[d]_[and] _**[ χ]**_ 1[= [] _**[x]**[⊤]_ 1[(] _[t]_[)] _[,]_ _**[ x]**[⊤]_ 1[(] _[t][ −] td_ ) _,_ _**x**[⊤]_ 1[(] _[t][−]_[2] _[t][d]_[)] _[,]_ _**[ τ]**[ ⊤]_ 2[]] _[⊤]_[.][The][control][parameters][are][set][as] _ρ_ = 200 _, kw_ = 0 _._ 01, _**K**[′]_ 1[=][diag][([][1] _[.]_[2] _[,]_[ 1] _[.]_[2] _[,]_[ 2] _[.]_[0] _[,]_[ 0] _[.]_[8][])] _[,]_ _**[K]**_[2][=] 

TABLE I 

QUANTITATIVE DATA OF TRACKING ERRORS FOR EXP1 

|**Exp1**||**Max**<br>**RMS**<br>_e_dis (m) _eψ_ (rad) _e_dis (m) _eψ_ (rad)|
|---|---|---|
|Proposed method<br>PD method with<br>ESN-based observer <br>PD method|**Test1** <br>**Test2**|0.1448<br>0.0322<br>0.0135<br>0.0032<br> 0.0678<br>0.0114<br>0.0318<br>0.0017<br> 0.3275<br>0.1244<br>0.0306<br>0.0072<br> 0.4500<br>0.0274<br>0.1380<br>0.0059<br> 0.3058<br>0.1775<br>0.0143<br>0.0046<br> 0.4753<br>0.0515<br>0.2500<br>0.0059|
||**Test1** <br> **Test2**||
||**Test1** <br>**Test2**||



diag([1 _._ 8 _,_ 1 _._ 8 _,_ 3 _._ 0 _,_ 1 _._ 4]) _,_ _**K** p_ = diag([1 _._ 4 _,_ 1 _._ 4 _,_ 1 _._ 2 _,_ 5]) _,_ _**K** d_ = diag([2 _._ 6 _,_ 2 _._ 6 _,_ 2 _._ 0 _,_ 7 _._ 5]). 

## _B. Experiment Group 1 (Basic Performance)_ 

_Test 1 (Circle trajectory):_ The first experiment aims to verify the basic control performance, with the desired trajectories of _**η**_ selected as follows: 

**==> picture [144 x 61] intentionally omitted <==**

where _ψ_ 0 indicates the yaw of the blimp before takeoff. The initial conditions are given as _**x**_ 10 = [1 _._ 3 _, −_ 0 _._ 1 _, −_ 0 _._ 06 _, −_ 0 _._ 03] _[⊤]_ , _**x**_ ˆ 20 = [0 _,_ 0 _,_ 0 _,_ 0] _[⊤]_ . Fig. 4 provides the experimental results of both the proposed controller and the comparative ones, which illustrates the trajectories, tracking errors, and control inputs of the blimp. The indices on the maximum and root mean square (rms) values of trajectory tracking errors are presented in Table I. From Table I, it can be seen that the maximum displacement error ( _edis_ ) achieved by the proposed method (0.1448 m) is only about 43.5% of the error from the PD method with ESN-based state observer (0.3275 m) and about 47.5% of the error from the PD method (0.3058 m). In addition, the maximum yaw error ( _eψ_ ) achieved by the proposed method (0.0322 rad) is only about 25.6% of the error from the PD method with ESN-based state observer (0.1244 rad) and about 18.2% of the error from the PD method (0.1775 rad). As can be seen from the results, all methods can make the blimp fly along the desired trajectory, and the proposed method successfully keeps the tracking errors within the specified limits, while the other two methods fail to do so. 

_Test 2 (Square trajectory):_ To further validate the performance of the designed controller, a square trajectory with a side length of 1.6 m is selected as the desired trajectory. The initial and the preset final conditions are set as _**x**_ 10 = [0 _,_ 0 _,_ 0 _,_ 0 _._ 03] _[⊤]_ and _**x**_ 1 _d_ = [0 _._ 8 _,_ 0 _._ 8 _,_ 1 _._ 0 _,_ 0 _._ 03] _[⊤]_ , respectively. This test confirms that the controller can start the blimp from a stationary state, execute sharp turns, and smoothly reach the predetermined final state. The experimental results of the trajectory tracking tests are shown in Fig. 5. The quantitative data of tracking errors are presented in Table I. From Fig. 5(b), it can be observed that all the tracking errors are kept within the time-varying constraints. 

Authorized licensed use limited to: NANKAI UNIVERSITY. Downloaded on February 09,2026 at 05:28:07 UTC from IEEE Xplore.  Restrictions apply. 

DONG et al.: ADAPTIVE OUTPUT FEEDBACK TRAJECTORY TRACKING CONTROL OF INDOOR BLIMP 

3967 

**==> picture [219 x 536] intentionally omitted <==**

Fig. 4. Results for Exp1-T1. (a) Trajectories of the blimp. (b) Trajectory tracking errors of the blimp. (c) Control inputs. 

Compared with the other two methods, the proposed method has significantly smaller tracking errors, thus further emphasizing the robustness and accuracy of the controller. 

## _C. Experiment Group 2 (Robustness Tests)_ 

_Test 1 (External disturbances test):_ In this test, a continuous wind disturbance is applied to the system via an air blower, aiming to validate the control performance in a more realistic 

**==> picture [218 x 547] intentionally omitted <==**

Fig. 5. Results for Exp1-T2. (a) Trajectories of the blimp. (b) Trajectory tracking errors of the blimp. (c) Control inputs. 

environment with external disturbances. The desired trajectories are chosen as follows: 

**==> picture [152 x 61] intentionally omitted <==**

Authorized licensed use limited to: NANKAI UNIVERSITY. Downloaded on February 09,2026 at 05:28:07 UTC from IEEE Xplore.  Restrictions apply. 

IEEE TRANSACTIONS ON INDUSTRIAL ELECTRONICS, VOL. 72, NO. 4, APRIL 2025 

3968 

TABLE II 

QUANTITATIVE DATA OF TRACKING ERRORS FOR EXP2-T1 

||**Max**<br>**RMS**|
|---|---|
|**Exp2-T1**|_e_dis (m) _eψ_ (rad) _e_dis (m) _eψ_ (rad)|
|Proposed method<br>PD method with<br>ESN-based observer <br>PD method|0.1837<br>0.0133<br>0.0527<br>0.0027<br> 0.1974<br>0.0206<br>0.0729<br>0.0101<br>0.4607<br>0.0315<br>0.1924<br>0.0063|
|TABLE III<br>RECOVERYTIMES FORCHANGES IN<br>BLIMPMASS<br><br>||
|**Exp2–T2** Proposed<br>Method<br>PD Method With<br>ESN-Based Observer PD Method||
|_tr_1 (_s_)<br>12.96<br>13.56<br>/<br>_tr_2 (_s_)<br>8.57<br>10.68<br>/||



The initial conditions are given as _**x**_ 10 = [0 _,_ 0 _,_ 0 _, −_ 0 _._ 03] _[⊤]_ , _**x**_ ˆ 20 = [0 _,_ 0 _,_ 0 _,_ 0] _[⊤]_ . Experimental results are illustrated in Fig. 6. The quantitative data of tracking errors are presented in Table II, which shows that the proposed method can reduce tracking errors when compared with the other methods. Fig. 6(d) indicates that the unknown disturbances can be estimated by the observer and compensated in the control inputs. This effective compensation for disturbances is evident, as the curves demonstrate that only the proposed method consistently keeps the tracking errors within a narrow range, even in the presence of external wind disturbances. In contrast, the tracking errors of the other two approaches tend to surpass the prescribed constraints, which shows that the proposed method reduces tracking errors compared with the other two methods. Additionally, if the PD method is used, there will be a steady-state error in the _ez_ component after the blimp stabilizes. 

_Test 2 (Model parameter uncertainties test):_ The effectiveness of the proposed control method is further validated by adjusting the model parameters in this part. Three seconds after the blimp reaches the anticipated altitude, its total mass is increased to 0 _._ 645 kg by suspending a 60 g weight to the gondola. Subsequently, the blimp then flies along the desired trajectory to spatial coordinates [0 _._ 8 _,_ 0 _._ 8 _,_ 1] _[⊤]_ , with the weight being unloaded from the gondola at 64 s. The recovery times for changes in blimp mass are shown in Table III. It is noteworthy that the altitude of the blimp will not recover after adding the payload of the gondola if the PD method is employed. For this reason, the recovery times for changes in blimp mass are omitted from Table III. The experimental results are shown in Fig. 7. Specifically, Fig. 7(c) illustrates that the disturbances caused by changes in model parameters can be successfully estimated by the ESN-based state observer. The proposed approach demonstrates minimal variation in state and the shortest recovery time following adjustments to the total mass of the blimp in contrast to the other two methods. 

**==> picture [200 x 638] intentionally omitted <==**

Fig. 6. Results for Exp2–T1. (a) Trajectories of the blimp. (b) Trajectory tracking errors of the blimp. (c) Control inputs. (d) Observer outputs. 

Authorized licensed use limited to: NANKAI UNIVERSITY. Downloaded on February 09,2026 at 05:28:07 UTC from IEEE Xplore.  Restrictions apply. 

DONG et al.: ADAPTIVE OUTPUT FEEDBACK TRAJECTORY TRACKING CONTROL OF INDOOR BLIMP 

3969 

**==> picture [235 x 649] intentionally omitted <==**

Fig. 7. Results for Exp2–T2. (a) Trajectory tracking errors of the blimp. (b) Control inputs. (c) Observer outputs. 

## V. CONCLUSION 

This article has developed an unmanned indoor blimp system and studies its robust trajectory tracking problem. Utilizing output feedback signals, the ESN-based state observer has been designed to deal with unknown disturbances and unreliable velocity measurements. Furthermore, the TABLF method has been utilized to keep the tracking errors within specified ranges. The stability of the closed-loop system has been guaranteed by proving that all signals are ultimately uniformly bounded through the construction of suitable Lyapunov candidate functions. Comparative experimental results have shown that even without additional external disturbances, the maximal tracking error of the proposed approach is only about 18.2% of that of the comparative methods. This has demonstrated the effectiveness and robustness of the proposed method. In the ensuing research, we will consider adding sensors such as light detection and ranging (LiDAR) in the gondola to enhance the accuracy of positioning. Meanwhile, the unique characteristics of blimps will be leveraged to broaden their practical applications in areas such as human–robot interaction and unknown environment exploration. 

## ACKNOWLEDGMENT 

The authors would like to thank the Associate Editor and all reviewers for the professional suggestions and comments, which have greatly improved the quality of the presentation. 

## REFERENCES 

- [1] M. Xu, A. Hu, and H. Wang, “Image-based visual impedance force control for contact aerial manipulation,” _IEEE Trans. Automat. Sci. Eng._ , vol. 20, no. 1, pp. 518–527, Jan. 2023. 

- [2] X. Liang, Z. Zhang, H. Yu, Y. Wang, Y. Fang, and J. Han, “Antiswing control for aerial transportation of the suspended cargo by dual quadrotor UAVs,” _IEEE/ASME Trans. Mechatron._ , vol. 27, no. 6, pp. 5159– 5172, Dec. 2022. 

- [3] Q. Fang, P. Mao, L. Shen, and J. Wang, “A global fast terminal sliding mode control for trajectory tracking of unmanned aerial manipulation,” _Meas. Control_ , vol. 56, no. 3-4, pp. 763–776, 2023. 

- [4] S. Bu, L. Yan, X. Gao, P. Zhao, and C. K. Lim, “Vision-guided manipulator operating system based on CSRT algorithm,” _Int. J. Hydromechatron._ , vol. 5, no. 3, pp. 260–274, 2022. 

- [5] Q. Tao, J. Wang, Z. Xu, T. X. Lin, Y. Yuan, and F. Zhang, “Swingreducing flight control system for an underactuated indoor miniature autonomous blimp,” _IEEE/ASME Trans. Mechatron._ , vol. 26, no. 4, pp. 1895–1904, Aug. 2021. 

- [6] H. Cheng and F. Zhang, “RGBlimp-Q: Robotic gliding blimp with moving mass control based on a bird-inspired continuum arm,” 2024, _arXiv:2406.10810_ . 

- [7] S. Sharma, M. Verhoeff, F. Joosen, R. Venkatesha Prasad, and S. Hamaza, “A morphing quadrotor-blimp with balloon failure resilience for mobile ecological sensing,” _IEEE Robot. Automat. Lett._ , vol. 9, no. 7, pp. 6408–6415, Jul. 2024. 

- [8] C. Lu et al., “A heterogeneous unmanned ground vehicle and blimp robot team for search and rescue using data-driven autonomy and communication-aware navigation,” _Field Robot._ , vol. 2, pp. 557– 594, May 2022. 

- [9] M. Hou, Q. Tao, P. Varnell, and F. Zhang, “Modeling pointing tasks in human-blimp interactions,” in _Proc. IEEE Conf. Control Technol. Appl. (CCTA)_ , 2019, pp. 73–78. 

- [10] A. Van Asares, P. S. Ko, J. S. Minlay, B. R. Sarmiento, and A. Chua, “Design of an unmanned aerial vehicle blimp for indoor applications,” _Int. J. Mech. Eng. Robot. Res._ , vol. 8, no. 1, pp. 157–161, 2019. 

- [11] V. Mai et al., “Local positioning system using UWB range measurements for an unmanned blimp,” _IEEE Robot. Automat. Lett._ , vol. 3, no. 4, pp. 2971–2978, Oct. 2018. 

Authorized licensed use limited to: NANKAI UNIVERSITY. Downloaded on February 09,2026 at 05:28:07 UTC from IEEE Xplore.  Restrictions apply. 

IEEE TRANSACTIONS ON INDUSTRIAL ELECTRONICS, VOL. 72, NO. 4, APRIL 2025 

3970 

- [12] M. Pellegrino, “Design and control of a miniature indoor blimp: A safer, quieter, and resilient alternative for indoor exploration and surveillance,” Ph.D. dissertation, Politecnico di Torino, Turin, Italy, 2023. 

- [13] G. Gorjup and M. Liarokapis, “A low-cost, open-source, robotic airship for education and research,” _IEEE Access_ , vol. 8, pp. 70713– 70721, 2020. 

- [14] P. González, W. Burgard, R. Sanz Domínguez, and J. López Fernández, “Developing a low-cost autonomous indoor blimp,” _J. Phys. Agents_ , vol. 3, no. 1, pp. 43–52, Jan. 2009. 

- [15] S. H. Song, G. Y. Yeon, H. W. Shon, and H. R. Choi, “Design and control of soft unmanned aerial vehicle “S-CLOUD”,” _IEEE/ASME Trans. Mechatron._ , vol. 26, no. 1, pp. 267–275, Feb. 2021. 

- [16] S. H. Song and H. R. Choi, “Design, control and implementation of torus-type omniorientational blimp with tilting actuators,” _IEEE Access_ , vol. 9, pp. 147985–147993, 2021. 

- [17] Y. T. Liu, E. Price, M. J. Black, and A. Ahmad, “Deep residual reinforcement learning based autonomous blimp control,” in _Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)_ , 2022, pp. 12566–12573. 

- [18] Y. T. Liu, N. Singh, and A. Ahmad, “Adaptive reinforcement learning for robot control,” 2024, _arXiv:2404.18713_ . 

- [19] A. Saeed, L. Wang, Y. Liu, M. Z. Shah, and Z. Y. Zuo, “Modeling and control of unmanned finless airship with robotic arms,” _ISA Trans._ , vol. 103, pp. 103–111, Aug. 2020. 

- [20] Y. Wang, G. Zheng, D. Efimov, and W. Perruquetti, “Disturbance compensation based controller for an indoor blimp robot,” _Robot. Auton. Syst._ , vol. 124, 2020, Art. no. 103402. 

- [21] M. Zhu, S. Yu, and Z. Zheng, “Model predictive control for path following of stratospheric airship with magnitude and rate constrains of rudder,” in _Proc. 27th Chin. Control Decis. Conf. (CCDC)_ , Piscataway, NJ, USA: IEEE Press, 2015, pp. 3444–3449. 

- [22] W. He, C. Xue, X. Yu, Z. Li, and C. Yang, “Admittance-based controller design for physical human–Robot interaction in the constrained task space,” _IEEE Trans. Automat. Sci. Eng._ , vol. 17, no. 4, pp. 1937–1949, Oct. 2020. 

- [23] C. Zhu, Y. Jiang, and C. Yang, “Fixed-time neural control of robot manipulator with global stability and guaranteed transient performance,” _IEEE Trans. Ind. Electron._ , vol. 70, no. 1, pp. 803–812, Jan. 2023. 

- [24] Y. Zhang, L. Kong, S. Zhang, X. Yu, and Y. Liu, “Improved sliding mode control for a robotic manipulator with input deadzone and deferred constraint,” _IEEE Trans. Syst., Man, Cybern.: Syst._ , vol. 53, no. 12, pp. 7814–7826, Dec. 2023. 

- [25] Z. Cuan and D.-W. Ding, “Fixed-time adaptive fuzzy tracking control for high-order uncertain nonlinear cyber-physical systems under malicious attacks,” _IEEE Trans. Syst., Man, Cybern.: Syst._ , vol. 54, no. 4, pp. 2379–2388, Apr. 2024. 

- [26] S. Li, L. Ding, H. Gao, Y.-J. Liu, L. Huang, and Z. Deng, “Adaptive fuzzy finite-time tracking control for nonstrict full states constrained nonlinear system with coupled dead-zone input,” _IEEE Trans. Cybern._ , vol. 52, no. 2, pp. 1138–1149, Feb. 2022. 

- [27] H. Jaeger, “The echo’ state approach to analysing and training recurrent neural networks-with an erratum note,” German Nat. Res. Center Inf. Technol., Bonn, Germany, GMD Tech. Rep., 2001, vol. 148, no. 34, p. 13. 

- [28] C. Sun, M. Song, S. Hong, and H. Li, “A review of designs and applications of echo state networks,” 2020, _arXiv:2012.02974_ . 

- [29] L. Zhang, W.-W. Che, B. Chen, and C. Lin, “Adaptive fuzzy outputfeedback consensus tracking control of nonlinear multiagent systems in prescribed performance,” _IEEE Trans. Cybern._ , vol. 53, no. 3, pp. 1932– 1943, Mar. 2023. 

- [30] G. Cui, J. Yu, and P. Shi, “Observer-based finite-time adaptive fuzzy control with prescribed performance for nonstrict-feedback nonlinear systems,” _IEEE Trans. Fuzzy Syst._ , vol. 30, no. 3, pp. 767–778, Mar. 2022. 

- [31] H. Xu, S. Liu, S. Zhao, and J. Wang, “Distributed control for a class of nonlinear systems based on distributed high-gain observer,” _ISA Trans._ , vol. 138, pp. 329–340, Jul. 2023. 

- [32] L. Kong, W. He, Z. Liu, X. Yu, and C. Silvestre, “Adaptive tracking control with global performance for output-constrained MIMO nonlinear systems,” _IEEE Trans. Autom. Control_ , vol. 68, no. 6, pp. 3760–3767, Jun. 2023. 

- [33] M. Khosravi, C. König, M. Maier, R. S. Smith, J. Lygeros, and A. Rupenyan, “Safety-aware cascade controller tuning using constrained Bayesian optimization,” _IEEE Trans. Ind. Electron._ , vol. 70, no. 2, pp. 2128–2138, Feb. 2023. 

- [34] Y. Ren, Z. Zhao, C. Zhang, Q. Yang, and K.-S. Hong, “Adaptive neural-network boundary control for a flexible manipulator with input constraints and model uncertainties,” _IEEE Trans. Cybern._ , vol. 51, no. 10, pp. 4796–4807, Oct. 2021. 

- [35] X.-Y. Zhang, X. Xie, Y.-J. Liu, and J. Sun, “Integral barrier Lyapunov function-based adaptive event-triggered control of flexible riser systems,” _IEEE Trans. Automat. Sci. Eng._ , early access, Jan. 15, 2024, doi: 10.1109/TASE.2024.3351739. 

- [36] G. Shen, P. Huang, Z. Ma, F. Zhang, and Y. Xia, “Dynamic eventbased adaptive fixed-time control for uncertain strict-feedback nonlinear systems with state constraints,” _IEEE Trans. Cybern._ , vol. 54, no. 8, pp. 4630–4642, Aug. 2024, doi: 10.1109/TCYB.2023.3293466. 

- [37] Z. Xu, C. Sun, X. Hu, Q. Liu, and J. Yao, “Barrier Lyapunov functionbased adaptive output feedback prescribed performance controller for hydraulic systems with uncertainties compensation,” _IEEE Trans. Ind. Electron._ , vol. 70, no. 12, pp. 12500–12510, Dec. 2023. 

- [38] Y. Zhang, W. Wu, and W. Zhang, “Noncooperative game-based cooperative maneuvering of intelligent surface vehicles via accelerated learning-based neural predictors,” _IEEE Trans. Intell. Vehicles_ , vol. 8, no. 3, pp. 2212–2221, Mar. 2023. 

- [39] W. Wu, R. Ji, W. Zhang, and Y. Zhang, “Transient-reinforced tunnel coordinated control of underactuated marine surface vehicles with actuator faults,” _IEEE Trans. Intell. Transp. Syst._ , vol. 25, no. 2, pp. 1872–1881, Feb. 2024. 

- [40] A. J. Calise, N. Hovakimyan, and M. Idan, “Adaptive output feedback control of nonlinear systems using neural networks,” _Automatica_ , vol. 37, no. 8, pp. 1201–1211, 2001. 

- [41] Z. Peng, D. Wang, W. Wang, and L. Liu, “Containment control of networked autonomous underwater vehicles: A predictor-based neural DSC design,” _ISA Trans._ , vol. 59, pp. 160–171, Nov. 2015. 

- [42] H. Zhou and S. Tong, “Adaptive neural network event-triggered outputfeedback containment control for nonlinear MASs with input quantization,” _IEEE Trans. Cybern._ , vol. 53, no. 11, pp. 7406–7416, Nov. 2023. 

- [43] X. Liu, L. Qiu, Y. Fang, K. Wang, Y. Li, and J. RodrÃguez, “Finite control-set learning predictive control for power converters,” _IEEE Trans. Ind. Electron._ , vol. 71, no. 7, pp. 8190–8196, Jul. 2024. 

- [44] Z. Liu et al., “B-spline wavelet neural-network-based adaptive control for linear-motor-driven systems via a novel gradient descent algorithm,” _IEEE Trans. Ind. Electron._ , vol. 71, no. 2, pp. 1896–1905, Feb. 2024. 

- [45] Y. Salmanpour, M. M. Arefi, A. Khayatian, and S. Yin, “Observerbased fault-tolerant finite-time control of nonlinear multiagent systems,” _IEEE Trans. Neural Netw. Learn. Syst._ , early access, Jun. 20, 2023, doi: 10.1109/TNNLS.2023.3279890. 

- [46] A. Isaly, O. S. Patil, H. M. Sweatland, R. G. Sanfelice, and W. E. Dixon, “Adaptive safety with a RISE-based disturbance observer,” _IEEE Trans. Autom. Control_ , vol. 69, no. 7, pp. 4883–4890, Jul. 2024. 

**==> picture [73 x 91] intentionally omitted <==**

**Jinyang Dong** received the B.S. degree in automation in 2023 from Nankai University, Tianjin, China, where he is currently working toward the Ph.D. degree in artificial intelligence with the Institute of Robotics and Automatic Information System. 

His research interest includes nonlinear control of indoor unmanned blimp. 

**==> picture [73 x 91] intentionally omitted <==**

**Hai Yu** received the B.S. degree in automation from Jilin University, Changchun, China, in 2020. He is currently working toward the Ph.D. degree in control science and engineering with the Institute of Robotics and Automatic Information System, Nankai University, Tianjin, China. 

His research interest includes nonlinear control of unmanned aerial vehicles. 

Authorized licensed use limited to: NANKAI UNIVERSITY. Downloaded on February 09,2026 at 05:28:07 UTC from IEEE Xplore.  Restrictions apply. 

DONG et al.: ADAPTIVE OUTPUT FEEDBACK TRAJECTORY TRACKING CONTROL OF INDOOR BLIMP 

3971 

**Biao Lu** (Member, IEEE) received the B.S. degree in intelligence science and technology from Nankai University, Tianjin, China, in 2015, and the Ph.D. degree in control theory and applications from the Institute of Robotics and Automatic Information System, Nankai University, in 2020. 

He is currently an Associate Professor with the Institute of Robotics and Automatic Information System (IRAIS), Nankai University. His research interests include motion planning and nonlinear control of various underactuated systems. 

**Huawang Liu** received the B.S. degree in intelligence science and technology from the Hebei University of Technology, Tianjin, China, in 2014, and the M.S. degree in electrical and electronics engineering from the University of Melbourne, Melbourne, VIC, Australia, in 2019. 

**Yongchun Fang** (Senior Member, IEEE) received the B.S. and M.S. degrees in control theory and applications from Zhejiang University, Hangzhou, China, in 1996 and 1999, respectively, and the Ph.D. degree in electrical engineering from Clemson University, Clemson, SC, USA, in 2002. 

From 2002 to 2003, he was a Postdoctoral Fellow with Sibley School of Mechanical and Aerospace Engineering, Cornell University, Ithaca, NY, USA.He is currently a Professor with the Institute of Robotics and Automatic Information Systems, Nankai University, Tianjin, China. His research interests include nonlinear control, visual servoing, control of underactuated systems, and atomic force microscopy (AFM)-based nanosystems. 

Dr. Fang received the National Science Fund for Distinguished Young Scholars of China. He was an Associate Editor of the _ASME Journal of Dynamic Systems, Measurement, and Control._ 

He is currently a Laboratory Technician with the Institute of Robotics and Automatic Information Systems, Nankai University, Tianjin. His research interests include embedded hardware, firmware for crane, and unmanned aerial vehicle systems. 

Authorized licensed use limited to: NANKAI UNIVERSITY. Downloaded on February 09,2026 at 05:28:07 UTC from IEEE Xplore.  Restrictions apply. 

