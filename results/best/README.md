## Best
$K_p = 2 * \begin{bmatrix}1 & 0 & 0 \\
                           0 & 1 & 0 \\
                           0 & 0 & 1 \end{bmatrix}$

$K_i = 0.5 * \begin{bmatrix}1 & 0 & 0 \\
                           0 & 1 & 0 \\
                           0 & 0 & 1 \end{bmatrix}$


This is a feedforward + PI controller. The error converges to 0 before the end of the first trajectory segment. Additionally, the starting config vector is
$[0, -0.5, -0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]$. Below, I plot the error over time.

![alt text](errors.png "Errors over time")
