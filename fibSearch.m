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


%search for the minimum of a convex function
function [minArg,x,nEvals,solnRecord] = fibSearch(func,maxRange,epsilon)

    solnRecord = [];

    %generate the fibonnacci numbers
    fiba = 0;
    fibb = 1;
    fibNums = [fiba fibb];
    while fibb < maxRange
        tmp = fibb;
        fibb = fiba + fibb;
        fiba = tmp;
        fibNums = [fibNums fibb];
    end

    k = numel(fibNums);

    a = 1;
    b = maxRange;
    c = a + floor((1 - fibNums(k-1) / fibNums(k)) * (b - a));
    d = a + floor((fibNums(k-1) / fibNums(k)) * (b - a));

    k = k - 1;

    %eval the function at these points
    [fc,x] = func(c);
    [fd,x] = func(d);
    nEvals = 2;    
    
    solnRecord = [solnRecord; c fc];
    solnRecord = [solnRecord; d fd];

    %check if we should next search the lower or upper half of the array
    while abs(c - d) > epsilon
        
        if fc >= fd
            %min must be in upper half
            a = c;
            c = d;            
            fc = fd;            
            d = a + floor((fibNums(k-1) / fibNums(k)) * (b - a));           
            [fd,x] = func(d);
            solnRecord = [solnRecord; d fd];
            
            %find a new fd
            %keep fc the same            
        else
            %min must be in lower half            
            b = d;
            d = c;
            fd = fc;            
            c = a + floor((1 - fibNums(k-1) / fibNums(k)) * (b - a));
            [fc,x] = func(c);  
            solnRecord = [solnRecord; c fc];
        end
        
        k = k - 1;
        nEvals = nEvals + 1;
        
    end
    
    minArg = c;
    
end

%this is the basic version of this function - just used to test that we are
%doing things in the right way
function test()
        
    f = 1:100;
    f = (f - 25).^2;

    plot(f)

    %search for the minimum of x

    %generate the fibonnacci numbers
    fiba = 0;
    fibb = 1;
    fibNums = [fiba fibb];
    while fibb < numel(f)
        tmp = fibb;
        fibb = fiba + fibb;
        fiba = tmp;
        fibNums = [fibNums fibb];
    end

    k = numel(fibNums);

    a = 1;
    b = numel(f) - 1;
    c = a + floor((1 - fibNums(k-1) / fibNums(k)) * (b - a));
    d = a + floor((fibNums(k-1) / fibNums(k)) * (b - a));

    k = k - 1;

    %fa = f(a);
    %fb = f(b);

    %eval the function at these points
    fc = f(c);
    fd = f(d);

    plot(f);
    hold on;
    scatter(c,f(c),'r');
    scatter(d,f(d),'b');
    hold off;

    %check if we should next search the lower or upper half of the array
    while abs(c - d) > 1
        
        if fc >= fd
            %min must be in upper half
            a = c;
            c = d;            
            fc = fd;            
            d = a + floor((fibNums(k-1) / fibNums(k)) * (b - a));           
            fd = f(d);
            
            %find a new fd
            %keep fc the same            
        else
            %min must be in lower half            
            b = d;
            d = c;
            fd = fc;            
            c = a + floor((1 - fibNums(k-1) / fibNums(k)) * (b - a));
            fc = f(c);            
        end
        
        hold on;
        scatter(c,f(c),'r');
        scatter(d,f(d),'b');
        hold off;
        
        k = k - 1;
        
    end
end
