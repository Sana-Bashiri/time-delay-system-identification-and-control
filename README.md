# System Identification and PID Control of a Time-Delay Process

MATLAB and Simulink study of reduced-order process modeling, relay-feedback identification, PID tuning, and Smith predictor compensation for a fourth-order plant with transport delay.

The project compares three-parameter and four-parameter process approximations, evaluates several PID tuning procedures, and examines nominal step-reference responses with and without model-based delay compensation.

## Process Model

The original plant is

$$
G(s)=\frac{1000e^{-2.5s}}{s^4+126s^3+2725s^2+12600s+1000}.
$$

Its DC gain is one, and its transport delay is 2.5 s.

## Methods

1. **Three-parameter identification:** Estimate a first-order plus dead-time (FOPDT) model from the maximum-slope tangent of the unit-step response.
2. **Four-parameter identification:** Fit a second-order plus dead-time (SOPDT) model by minimizing the squared step-response error. Parameters are optimized in logarithmic coordinates to preserve positivity.
3. **Model comparison:** Compare the original plant and both reduced-order models in Simulink.
4. **Relay feedback:** Estimate the ultimate frequency and ultimate gain from sustained relay-induced oscillation.
5. **PID design:** Evaluate time-domain, frequency-domain, and generalized Ziegler-Nichols tuning, together with the procedure labeled lambda tuning in the report. Compare calculated starting gains with manually adjusted gains.
6. **Smith prediction:** Evaluate delay compensation with full process dynamics and a reduced FOPDT internal model.

The MATLAB identification scripts use a 60 s response window with a sampling interval of 0.01 s.

## Reported Results

### Reduced-order models

The identified FOPDT model is

$$
G_3(s)=\frac{0.990211e^{-2.698647s}}{12.982952s+1}.
$$

The identified SOPDT model is

$$
G_4(s)=\frac{0.999998e^{-2.551851s}}{(12.380540s+1)(0.167542s+1)}.
$$

| Model | Gain | Time constant(s), s | Delay, s |
| --- | ---: | --- | ---: |
| FOPDT | 0.990211 | T = 12.982952 | 2.698647 |
| SOPDT | 0.999998 | T1 = 12.380540; T2 = 0.167542 | 2.551851 |

The SOPDT response closely follows the original response in the reported plot. Its larger time constant captures the dominant slow dynamics, while the smaller time constant represents a faster component.

**Notation:** The original filenames and plot labels use “3rd” and “4th” for the three- and four-parameter approximations. Their delay-free rational parts are first- and second-order, respectively. The original plant is fourth-order.

The FOPDT gain is estimated from the response at 60 s, so its value is a finite-horizon approximation to the exact plant DC gain of one.

### Relay-feedback estimates

| Quantity | Reported estimate |
| --- | ---: |
| Relay output levels | +1 and -1 |
| Output extrema | +0.189 and -0.192 |
| Ultimate period, Tu | 9.806 s |
| Ultimate angular frequency, omega_u | 0.6407 rad/s |
| Process magnitude at the ultimate point | 0.1496 |
| Ultimate gain, Ku | 6.6845 |

The magnitude estimate uses the relay describing-function approximation, with output half-amplitude `a = 0.1905` and relay half-amplitude `d = 1`:

$$
|G(j\omega_u)|\approx\frac{\pi a}{4d},\qquad
\omega_u=\frac{2\pi}{T_u},\qquad
K_u\approx\frac{1}{|G(j\omega_u)|}.
$$

### PID gains

The ideal controller parameters and parallel gains are related by

$$
C(s)=K_p\left(1+\frac{1}{T_i s}+T_d s\right),\qquad
P=K_p,\quad I=\frac{K_p}{T_i},\quad D=K_pT_d.
$$

| Tuning procedure | Calculated P | Calculated I | Calculated D | Adjusted P | Adjusted I | Adjusted D |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Time-domain Ziegler-Nichols | 5.8309 | 1.0803 | 7.8675 | 3.3 | 0.25 | 2.7 |
| Frequency-domain Ziegler-Nichols | 4.0107 | 0.8180 | 4.9159 | 2.5 | 0.4 | 3.5 |
| Generalized Ziegler-Nichols | 1.3286 | 0.1101 | 4.0088 | 2.9 | 0.25 | 2.0 |
| Reported lambda procedure | 6.730 | 1.100 | 6.367 | 3.7 | 0.4 | 3.0 |

The supplied PID comparison model stores the **adjusted** gains. Its derivative filter coefficient is `N = 10`.

The plots generally show reduced oscillation after manual adjustment. However, the frequency-domain tuning plots appear to end below unity; the report does not resolve this apparent tracking discrepancy. Numerical overshoot, settling-time, and error-integral comparisons are not tabulated.

The procedure labeled lambda tuning is documented through explicit formulas in the report; no separate lambda design parameter is specified.

### Smith predictor

The two Smith predictor configurations use the calculated gains `P = 6.730`, `I = 1.100`, `D = 6.367`, with `N = 10`.

The reported responses show modest overshoot and convergence toward the unit reference with both full and reduced internal models. These simulations illustrate nominal tracking behavior; they do not establish robustness to delay uncertainty or plant-model mismatch.

## Project Files

Keep the following files together in the repository root to use the commands and links below:

| File | Purpose |
| --- | --- |
| `matlab_part1_3rd.m` | FOPDT identification using the step-response tangent; plots the construction and prints the fitted parameters. |
| `matlab_part1_4th.m` | SOPDT identification using positive-parameter numerical optimization; plots the fitted response against the original plant. |
| `simulink_part2.slx` | Step-response comparison of the original plant, FOPDT model, and SOPDT model. |
| `simulink_part3_relay.slx` | Relay-feedback experiment for ultimate-point estimation. |
| `simulink_part4.slx` | Four PID feedback configurations containing the manually adjusted gains. |
| `simulink_part5.slx` | Smith predictor configurations with full and reduced internal models. |
| [English summary](english_project_summary.pdf) | Five-page English academic summary with figures reproduced from the original report. |
| [Original Persian report](final_project_report_gholipour.pdf) | Source calculations, model diagrams, and simulation figures. |

## Requirements

- MATLAB
- Simulink, for the `.slx` models
- Control System Toolbox, for transfer-function construction and step-response calculations

The SOPDT script uses MATLAB's `fminsearch` and does not call Optimization Toolbox functions. Use a MATLAB release that supports local functions in scripts and the plotting functions used by the code. The original MATLAB/Simulink release has not been verified.

## Running the Project

Open MATLAB and set the Current Folder to the directory containing the project files.

### 1. Identify the reduced-order models

Run each script separately:

```matlab
run('matlab_part1_3rd.m')
```

```matlab
run('matlab_part1_4th.m')
```

Each script clears the workspace and closes existing figures at startup. Review or save its output before running the next script. Small differences in fitted values can arise from numerical settings and software versions.

### 2. Inspect and simulate the models

```matlab
open_system('simulink_part2.slx')
open_system('simulink_part3_relay.slx')
open_system('simulink_part4.slx')
open_system('simulink_part5.slx')
```

Run each model using **Run** in Simulink, then open its Scope blocks to inspect the responses. The inspected models contain numerical plant and controller parameters in their blocks.

To simulate an individual model from the MATLAB Command Window:

```matlab
sim('simulink_part2.slx');
```

The stored PID comparison model contains the final adjusted gains. To examine the calculated starting designs, enter the corresponding P, I, and D values from the table above in the relevant PID blocks. Save a separate copy if you want to preserve the supplied settings.

## Interpretation and Limitations

- Results documented here are taken from the supplied report and checked against the source files; they have not been independently reproduced by executing MATLAB or Simulink.
- Model-fit quality and controller performance are primarily assessed through the reported response plots.
- The report does not provide a common numerical ranking based on settling time, overshoot, integrated error, or control effort.
- Disturbance rejection, measurement noise, actuator limits, and robustness to model or delay uncertainty are not evaluated.

The project demonstrates a complete simulation workflow from reduced-order identification to PID tuning and nominal delay compensation. Its results should be interpreted within the scope of the reported experiments.
