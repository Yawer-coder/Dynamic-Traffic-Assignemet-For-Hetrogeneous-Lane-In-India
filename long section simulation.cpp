// AW-Rascle model for heterogeneous traffic

#include<vector>
#include<fstream>
#include<iostream>
#include<cstdlib>
#include<stdio.h>
#include<math.h>
//#include "nlopt.hpp"
#include <bits/stdc++.h>	
#include<stdlib.h>
using namespace std;

int main(){
int g;
double dx,dt,W,w[6],a[6],d[6],L; 

//Vehicle dimensions - Arasan and Dhivya (2010)-Proceedings 24th European Conference on Modelling and Simulation 
/*a[1]=0.0018*0.0006; a[2]=0.0026*0.0014; a[3]=0.004*0.0019; a[4]=0.0085*0.0025; 
w[1]=0.0006; w[2]=0.0014; w[3]=0.0019; w[4]=0.0025;*/
a[1]=0.0018*0.0006; a[2]=0.0026*0.0014; a[3]=0.004*0.0016; a[4]=0.0065*0.0022; 
w[1]=0.0006; w[2]=0.0014; w[3]=0.0016; w[4]=0.002;
W=0.0075;
L=1.0;dx=0.02;

g=int(L/dx)+1;	

//dt=dx/Uf1[3];// for stable numerical scheme,simulation interval based on maximum free flow speed- cars (index 3) are having maximum free flow speed
dt=1.0/3600.0; 	

	
	int n=1800,x,t,c,o,y,T,i,z,r,m;	
	double AO[60],AO1[60],A[60], uf[6],tau[6], v[20], cjam[6], AOmax[6],  M1[6], M2[6], diffuao[6][60], diffuk[6][60],diffuaok[6][60];
	double k[6][60],q[6][60],u[6][60],Ui[6][60],Ui1[6][60],dk[6][60],dAO[60], AOk[6][60], op[60], AOnot[6][60], cavg[6];
	double inflow[6][1800],outflow[6][1800],initialveh[6],ue[6][60];
	

	// ******************Real outflow and speed values for calibration************************************************************************************

	ifstream initial("initialguess.txt");// outflow from the last section
	for(y=0;y<=15;y++){
		initial>>v[y];
	}
	initial.close();
		
	// getting paarmeter values from NLOPT algorithm
	c=1;y=0;
	while(c<=4)
	{		
	AOmax[c]=v[y]; uf[c]=v[y+1]; cjam[c]=v[y+2];tau[c]=v[y+3];y+=4;
	c++;
	}
	

	cout << "Program started...\n";

	//*********************Initial condition- densities and speeds of vehicle types for different sections at t=0****************************************************
	t=0;
	for (c=1;c<=4;c++)
		initialveh[c]=0.0;
	ifstream kinitial("initialdata.txt");
	x=0;
	while(x<=51){
		kinitial>>k[1][x]>>k[2][x]>>k[3][x]>>k[4][x];
				
		A[x]=(a[1]*k[1][x]+a[2]*k[2][x]+a[3]*k[3][x]+a[4]*k[4][x])/W;AO1[x]=A[x]; //cout<<AO[x]<<"\t";getch();
		if(A[x]<0.0019) A[x]=0.0019; if (A[x]>AOmax[1]) A[x]=AOmax[1];
		AOnot[1][x]=(a[2]*k[2][x]+a[3]*k[3][x]+a[4]*k[4][x])/W;
		AOnot[2][x]=(a[1]*k[1][x]+a[3]*k[3][x]+a[4]*k[4][x])/W;
		AOnot[3][x]=(a[1]*k[1][x]+a[2]*k[2][x]+a[4]*k[4][x])/W;
		AOnot[4][x]=(a[1]*k[1][x]+a[2]*k[2][x]+a[3]*k[3][x])/W;
		
		for(c=1;c<=4;c++) {			
			ue[c][x]=min(uf[c],max(3.0,(uf[c]*(1.0-exp(1.0-exp((cjam[c])*(AOmax[c]/(A[x])-1.0)))))));
			u[c][x]=ue[c][x];
			q[c][x]=k[c][x]*u[c][x];			
		}
		x++;
	}
	kinitial.close();	
	
	//for(x=0;x<=7;x++){	
	//	//if(x<=2) W=0.0075;else W=0.00375;
	//	W=0.0075;
	//	AO[x]=(a[1]*k[1][x]+a[2]*k[2][x]+a[3]*k[3][x]+a[4]*k[4][x])/W;AO1[x]=AO[x];			
	//}
	
	
	// **************************Simulating flow dynamics for next time intervals**************************************************************************
	ofstream data_w("dataawr.txt");
	ofstream wave("wavespeed.txt");
	ofstream speed_tw("TWspeed.txt");ofstream speed_thw("ThWspeed.txt");ofstream speed_car("CARspeed.txt");ofstream speed_ov("OVspeed.txt");
	ofstream density_tw("TWdensity.txt");ofstream density_thw("ThWdensity.txt");ofstream density_car("CARdensity.txt");ofstream density_ov("OVdensity.txt");
	ofstream occup("areaoccupancy.txt");
	ifstream flow("secinflow.txt");// read inflow to the section 
	
	t=1;
	
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$          SIMULATION       $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	while(t<=n){		
		
		//UPDATING THE PARAMETER VALUES***************************************************************************************************************************		
		for(c=1;c<=4;c++){
			// Calculation of state of each dx
			for(x=1;x<=51;x++){	

				M1[c]=exp((cjam[c])*(AOmax[c]/A[x]-1.0));
				M2[c]=exp(1-M1[c]);
				
				diffuk[c][x]=-1.0*(AOmax[c]*a[c]*cjam[c]*uf[c]*M1[c]*M2[c])/(W*A[x]*A[x]);
				diffuao[c][x]=-1.0*(AOmax[c]*cjam[c]*uf[c]*M1[c]*M2[c])/(A[x]*A[x]);
				diffuaok[c][x]=((AOmax[c]*AOmax[c]*a[c]*cjam[c]*cjam[c]*uf[c]*M1[c]*M2[c]))/(W*A[x]*A[x]*A[x]*A[x])-(AOmax[c]*AOmax[c]*a[c]*cjam[c]*cjam[c]*uf[c]*exp(cjam[c]*(AOmax[c]/A[x]-1)*2)*M2[c])/(W*A[x]*A[x]*A[x]*A[x]*A[x])+(AOmax[c]*a[c]*cjam[c]*uf[c]*M1[c]*M2[c]*2)/(W*A[x]*A[x]*A[x]);
			    //cavg[c]=max(0.0,min((dx/tau[c]),abs(diffuao[c][x]+k[c][x]*(diffuk[c][x]+diffuaok[c][x]))));
				cavg[c]=min(u[c][x],(u[c][x]+diffuao[c][x]+k[c][x]*(diffuk[c][x]+diffuaok[c][x])));
				
				//if(x==1)
					//Ui[c][x]=k[c][x]-(dt/dx)*(q[c][x]-q[c][x-1]);
				//else
					//Ui[c][x]=k[c][x]+(dt/dx)*k[c][x]*(u[c][x]-u[c][x+1])+(dt/dx)*u[c][x]*(k[c][x-1]-k[c][x]); 
					Ui[c][x]=k[c][x]-(dt/dx)*(k[c][x]*u[c][x+1]-k[c][x-1]*u[c][x]);
				
				if(cavg[c]>=0.0) 
				Ui1[c][x]=u[c][x]-(dt/dx)*u[c][x]*(u[c][x]-u[c][x-1])-(cavg[c]-u[c][x])*(diffuao[c][x]*(AOnot[c][x]-AOnot[c][x-1])+diffuk[c][x]*(k[c][x]-k[c][x-1]))+(dt/tau[c])*(ue[c][x]-u[c][x]);// speed
				else Ui1[c][x]=u[c][x]-(dt/dx)*u[c][x]*(u[c][x+1]-u[c][x])-(cavg[c]-u[c][x])*(diffuao[c][x]*(AOnot[c][x+1]-AOnot[c][x])+diffuk[c][x]*(k[c][x+1]-k[c][x]))+(dt/tau[c])*(ue[c][x]-u[c][x]);// speed
			   wave<<"\n"<<t<<"\t"<<x<<"\t"<<W<<"\t"<<c<<"\t"<<a[c]<<"\t"<<M1[c]<<"\t"<<M2[c]<<"\t"<<k[c][x]<<"\t"<<u[c][x]<<"\t"<<diffuk[c][x]<<"\t"<<diffuao[c][x]<<"\t"<<cavg[c]-u[c][x]<<"\t"<<cavg[c];		
			}
		}//*********************************************************************************************************************************************************************************

		
		
		//SUBSTITUTING THE UPDATED VALUES OF DENSITY AND SPEED**********************************************************************************************
		for(c=1;c<=4;c++){
			for(x=1;x<=50;x++){
				k[c][x]=max(0.0,Ui[c][x]);if(Ui1[c][x]<0.0)u[c][x]=ue[c][x];else u[c][x]=min(uf[c],max(1.0,Ui1[c][x]));
				//wave<<"\n"<<t<<"\t"<<x<<"\t"<<W<<"\t"<<c<<"\t"<<a[c]<<"\t"<<M1[c]<<"\t"<<M2[c]<<"\t"<<k[c][x]<<"\t"<<u[c][x]<<"\t"<<diffuk[c][x]<<"\t"<<diffuao[c][x]<<"\t"<<(u[c][x]-cavg[c]);		
			}

			k[c][51]=k[c][50]; u[c][51]=u[c][50];
			 // x=0 and x=19 are extra cells used for simulation
				
		}//*****************************************************************************************************************************	
		
		
		// UPDATING AO USING NEW DENSITY VALUES, CALCULATING NEW FLOW VALUES, AND FINDING ERRORS IN THIS TIME STEP****************************************************************************************************
		for(x=1;x<=51;x++){// calculate new AO for all sections, equilibrium speed for vehicle types and all errors
			
			dAO[x]=AO1[x];
			A[x]=(a[1]*k[1][x]+a[2]*k[2][x]+a[3]*k[3][x]+a[4]*k[4][x])/W;
			AOnot[1][x]=min((AOmax[1]-a[1]*k[1][x]/W),(a[2]*k[2][x]+a[3]*k[3][x]+a[4]*k[4][x])/W);
		    AOnot[2][x]=min((AOmax[1]-a[2]*k[2][x]/W),(a[1]*k[1][x]+a[3]*k[3][x]+a[4]*k[4][x])/W);
		    AOnot[3][x]=min((AOmax[1]-a[3]*k[3][x]/W),(a[1]*k[1][x]+a[2]*k[2][x]+a[4]*k[4][x])/W);
		    AOnot[4][x]=min((AOmax[1]-a[4]*k[4][x]/W),(a[1]*k[1][x]+a[2]*k[2][x]+a[3]*k[3][x])/W);
			
			if(A[x]<0.0019) A[x]=0.0019; 
			AO1[x]=A[x];//cout<<t<<"\t"<<x<<"\t"<< AO1[x]<<"\n";
			
			if(A[x]>=AOmax[1]){// maximum AO[x] is the AOmax of two wheeler
				for(c=1;c<=4;c++)	u[c][x]=0.0; 					
				A[x]=AOmax[1];AO1[x]=AOmax[1];// AO1[x] is stored as a copy of AO[x], since we may have to assign for each c, AO[x] can be maximum of AOmax[c]

			}
			dAO[x]=dAO[x]-AO1[x];

			for(c=1;c<=4;c++){// calculate equilibrium speed for each vehicle type
				dk[c][x]=dk[c][x]-k[c][x];
				if(A[x]>=AOmax[c]) u[c][x]=0.0;				
				if(A[x]<=0.0019) {u[c][x]=uf[c];ue[c][x]=uf[c];A[x]=0.0019;} 
				else 
				{ue[c][x]=min(uf[c],max(0.0,(uf[c]*(1.0-exp(1.0-exp(cjam[c]*(AOmax[c]/A[x]-1.0)))))));}	
				//if((k[c][x]<1.0)||(AO[x]==AOmax[c])||(u[c][x]==1.0)) u[c][x]=ue[c][x];
				q[c][x]=k[c][x]*u[c][x];
				//cout<<"\n"<<t<<"\t"<<x<<"\t"<<c<<"\t"<<AO[x]<<"\t"<< k[c][x]<<"\t"<< u[c][x]<<"\t"<<ue[c][x]<<"\t"<< q[c][x];
			}
			//getch();			
		}//****************************************************************************************************************************************************************************
		
		
		// INFLOW TO THE SECTION***************************************************************
		flow >>q[1][0]>>q[2][0]>>q[3][0]>>q[4][0]; //boundary condition - flow input
		for(c=1;c<=4;c++){
			//if(q[c][0]==0.0) q[c][0]=1.0;
			inflow[c][t]=q[c][0];u[c][0]=uf[c];//u[c][5]=uf[c];// cout<<"\n" <<inflow[c][t];getch();
			k[c][0]=q[c][0]/u[c][0];//q[c][5]=q[c][0];k[c][5]=k[c][0];
			ue[c][0]=uf[c];	//ue[c][5]=ue[c][0];		
		}//*************************************************************************************		
		
		
		// WRITNG OUTPUT FILES IF THE LAST ITERATION*******************************************************************************************
		
		for(x=0;x<=51;x++){
			data_w<<"\n"<<t <<"\t" << x<<"\t"<<A[x] <<"\t" << k[1][x]<<"\t"<<ue[1][x]<<"\t"<<u[1][x]<<"\t"<<q[1][x]<< "\t"<< k[2][x]<<"\t"<<ue[2][x]<<"\t"<<  u[2][x]<<"\t"<<q[2][x] 
			<< "\t" << k[3][x]<<"\t"<<ue[3][x]<<"\t"<< u[3][x]<<"\t"<<q[3][x] <<"\t" <<k[4][x]<< "\t" << ue[4][x]<<"\t"<< u[4][x]<<"\t"<<q[4][x] << "\n";
			
			speed_tw<<u[1][x]<<"\t";speed_thw<<u[2][x]<<"\t";speed_car<<u[3][x]<<"\t";speed_ov<<u[4][x]<<"\t";
			density_tw<<k[1][x]<<"\t";density_thw<<k[2][x]<<"\t";density_car<<k[3][x]<<"\t";density_ov<<k[4][x]<<"\t";
			occup<<A[x]<<"\t";
		
		}//************************************************************************************************************************************
	speed_tw<<"\n";	speed_thw<<"\n";speed_car<<"\n";speed_ov<<"\n";
	density_tw<<"\n";density_thw<<"\n";density_car<<"\n";density_ov<<"\n";
	occup<<"\n";

		t++;
	}	
	flow.close();
	wave.close();
	data_w.close();
	speed_tw.close();speed_thw.close();speed_car.close();speed_ov.close();
	density_tw.close();density_thw.close();density_car.close();density_ov.close();
	occup.close();
 }
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	
	
	