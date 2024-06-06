function [ q,Warning ] = Newton_Raphson_Method(FuncEval,q,time)
%
%Summary: This function solves a system of nonlinear equations
%         useing the Newton-Raphson method.
%
%Input:   q        - Estimate for the positions
%         FuncEval - Function for equations and derivatives
%
%Output:  q        - Corrected positions
%         Warning  - Exit condition of the NR iterative process
%                    Warning.Flag=1 - Convergence obtained
%                    Warning.Flag=0 - No convergence in max iter
%                    Warning.String - Non-convergence text
%
%Shared:  Parameter - Structured variables with NL parameters
%                     NLTolerance - Convergence tolerance
%                     NLMaxIter   - Maximum iteration number
%
% Dynamics of Mechanical Systems
% Version 2.0     March, 2022
%
%%
%... Access the global memory
global Parameter
%
%... Solve the nonlinear system of equations w/ Newton-Raphson
for iter = 1:Parameter.NLMaxIter
%
%... Evaluate constraint equations and Jacobian matrix
    [phi,Jac,~,~] = FuncEval(time,q,[]);
%
%... Evaluate the position correction
    dq = Jac\phi;
%
%... Correct positions
    q  = q-dq;
%
%... Check if solution is obtained
    if max(abs(dq))<Parameter.NLTolerance
        Warning.Flag = 1;
        break
    end
%
%... Check if maximum number of iterations is reached and warn
    if iter == Parameter.NLMaxIter
        NLMaxIter      = Parameter.NLMaxIter;
        Warning.String = ['Max iterations of NR (' ...
                          num2str(NLMaxIter) ...
                          ') reached'];
        Warning.Flag   = 0;
    end
end
%%
%... Finalize Newton_Raphson_Method function
end