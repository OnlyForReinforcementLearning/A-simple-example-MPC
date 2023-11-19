# A-simple-example-MPC
Use 'fmincon' and 'quadprog' to simulate MPC 

There are two different programs to show the MPC for tracking control for a linear system, and it is very suitable for new beginner who wants to learn MPC.

'MPC_QP' shows MPC for a linear system by using 'quadprog', and 'MPC_fmincon' shows MPC for a linear system by using 'fmincon'.

Note that some linear constraints of 'quadprog' or 'fmincon' don't contain model constraint.

‘fmincon’ is more suit for a MPC problem with nonlinear constraints. 

Hope these two matlab programs can help you! 


分别采用‘fmincon’和‘quadprog’实现了MPC代码。由于代码命令原因，这两份代码并没有系统模型约束。‘fmincon’更加适用于包含非线性约束的MPC问题。‘MPC_QP’利用‘quadprog’展示了MPC，‘MPC_fmincon’利用了‘fmincon’展示了MPC。

希望这两份代码可以帮助到你！
