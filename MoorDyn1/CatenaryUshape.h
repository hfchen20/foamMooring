/* 
  hfchen - 7/27/21
  
  The following functions solve catenary solution for a U-shaped line hanging on two supports
  of uneuqal heights, which is then used as a starting point for getting the initial line profiles.
  Called by  
  void Line::initializeLine(double* X );
*/


#include <stdio.h>
#include <cmath>
#include <vector>

double Calc_D(double a,  double L,  double h, double sgn)
// Calculates d from equation 11
{
    double q=2*sgn*sqrt(h*(a+h)*(L*L-a*a)); // + or - 2* the root used in (11)
    // return calculated d from eq (11)
    double d=((L*L-a*a)*(a+2*h)-L*q)/(a*a)*atanh(a*a/(L*(a+2*h)-q)); 
    
    return d;
}

double Solve_h(double a,  double L, double d)
// Routine to solve h from a, L and d
{
    double s=(L*L-a*a)/(2*a)*log((L+a)/(L-a));// Left or right of Y axis ?
    s = s<d ? -1:1;

    double MAXERR=1e-10; // Absolute precision of calculation
    int MAXIT=100;
    int n=1; // Iteration counter (quit if >MAXIT)
    
    double lower=0, upper=(L-a)/2; // h must be within this range
    double TV;
    while((upper-lower) > MAXERR && n<MAXIT) // Until range narrow enough or MAXIT
    {
        TV=(upper+lower)/2;
        if(Calc_D(a,L,TV,s)*s<d*s) 
            upper=TV; 
        else 
            lower=TV; // Narrow the range of possible h
        n=n+1;        
    }

    //Returns h (- signals right of Y axis)
    return s*TV;
}

// Defining variables:
//       a: height difference of the two support points
//       L: the total length of the rope
//       d: the horizontal distance between the two fixed points.
//       snodes: arclength along the line for each node

// 	int success = Catenary
// ( XF, ZF, UnstrLen, E*pi/4.*d*d, W , CB, Tol, &HF, &VF, &HA, &VA, N, snodes, Xl, Zl, Te);

int catUshapeSol(double a, double L, double d, double w, int N, vector<double>& snodes, vector<double>& Xl, vector<double>& Zl)
{    
    // h: the vertical distance from the lowest point of the rope to the lowest fixed point
    double h=Solve_h(a, L, d);

    h=abs(h);//assuming the lowest point is left of Y-axis
    
    double tmp=sqrt(h*(a+h)*(L*L-a*a));
    double L1=(h*L-tmp)/(-a);
    double miu=2*h/(L1*L1-h*h);//equals weight/force_x, inverse of Th/weight
    
    //x1 is the horizontal distance between the deepest point to the lowest pole
    double x1=asinh(miu*L1)/miu;

    double slope=sinh(miu*(0-x1));
    double H=w/miu;
    double V=H*slope;//horizontal and vertical (*negative*) tension at left end

    //s=0:L/N:L;// arclength
    for(int i=1; i<N; i++)
    {
        double ws = w*snodes[i];
        double t1 = (V+ws)/H;
        double t2 = V/H;
        
        // Xl[i]=H/w*(asinh((V+w*s)/H)-asinh(V/H));
        // Zl[i]=H/w*(sqrt(1+((V+w*s)/H)^2)-sqrt(1+(V/H)^2));
        
        Xl[i]=H/w*(asinh(t1)-asinh(V/H));
        Zl[i]=H/w*(sqrt(1+t1*t1)-sqrt(1+t2*t2));
    }

    return 1;     
}

/*
slope=sinh(miu*(x-x1));//slope and tension at each node
theta=atan(slope);
ten=H./abs(cos(theta));
*/