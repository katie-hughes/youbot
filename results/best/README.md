## Best
$K_p = 2 * \begin{bmatrix}1 & 0 & 0 \\
                           0 & 1 & 0 \\
                           0 & 0 & 1 \end{bmatrix}$

$K_i = 0.5 * \begin{bmatrix}1 & 0 & 0 \\
                           0 & 1 & 0 \\
                           0 & 0 & 1 \end{bmatrix}$

This is a feedforward + PI controller. The error converges to 0 by the end of the first motion. Additionally, the starting config vector is
$[0, -0.25, 0, 0, 0, 0, -1.57, 0, 0, 0, 0, 0, 0]$. Below, I plot the error over time.

![alt text](errors.png "Title")