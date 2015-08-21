% Copyright (c) 2015 Niall McLaughlin, CSIT, Queen's University Belfast, UK
% Contact: nmclaughlin02@qub.ac.uk
% If you use this code please cite:
% "Enhancing Linear Programming with Motion Modeling for Multi-target Tracking",
% N McLaughlin, J Martinez Del Rincon, P Miller, 
% IEEE Winter Conference on Applications of Computer Vision (WACV), 2015 
% 
% This software is licensed for research and non-commercial use only.
% 
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
% THE SOFTWARE.


%solve linear program
function [val,x] = minFunc(flow,costs,constraintsLeq,constraintsLegRHS,constraintsEq,constraintsEqRHS,lb,ub)

    constraintsEqRHSMod = constraintsEqRHS;
    constraintsEqRHSMod(constraintsEqRHSMod == -1) = flow;
    opts = optimoptions('linprog','Display','off');
    [x,fval,exitflag,output,lambda] = linprog(costs,constraintsLeq,constraintsLegRHS',constraintsEq,constraintsEqRHSMod',lb',ub',0,opts);
    val = costs * x;
    
end