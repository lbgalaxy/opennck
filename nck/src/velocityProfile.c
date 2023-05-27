/*****************************************************************//**
 * \file   velocityProfile.c
 * \brief  速度轮廓规划算法实现
 * 
 * \author galaxy
 * \date   March 2023
 *********************************************************************/
#include"velocityProfile.h"
#include<math.h>
#include<string.h>
#include<float.h>
 /**
  * \brief 梯形加减速规划.
  *
  * \param tp	梯形速度曲线结构体。
  * \param L	位移。
  * \param vs	初始速度。
  * \param vmax	最大速度限制。
  * \param ve	末速度。
  * \param amax	 加速度限制。
  * \param dmax   减速度限制。
  * \return      返回0执行成功，非零值执行失败。
  */
int calcTrapezoidalProfile(double L, double vs, double vmax, double ve, double amax, double dmax, TrapeProfile_t* tp)
{
	memset(tp, 0, sizeof(TrapeProfile_t));
	double vf = 0.0,  vc = vmax,  acc = amax, dec = -dmax;
	tp->acc = acc;
	tp->dec = dec;
	tp->L = L;
	//起步速度和终止速度不能大于匀速速度
	if (vs > vc)
		vs=vc;
	if (ve > vc)
		ve = vc;
	if (L < 1.0E-5)
	{
		return 0;
	}
	//4种情形
	vf = sqrt((-2.0 * acc * dec * L - vs * vs * dec + ve * ve * acc) / (acc - dec));
	if (vf > vc)//有匀速段
	{
		
		tp->vs = vs;
		tp->vc = vc;
		tp->ve = ve;
		tp->t1 = (vc - vs) / acc;
		tp->t3 = (ve - vc) / dec;
		tp->L1 = vs * tp->t1 + 0.5 * acc * tp->t1 * tp->t1;
		tp->L3 = vc * tp->t3 + 0.5 * dec * tp->t3 * tp->t3;
		tp->L2 = L - tp->L1 - tp->L3;
		tp->t2 = tp->L2 / vc;
		tp->t = tp->t1 + tp->t2 + tp->t3;
	}
	else//没有匀速段
	{
		if (vs < vf && vf < ve)//只有加速段
		{
			ve = sqrt(vs * vs + 2 * acc * L);
			vc = ve;
			tp->vs = vs;
			tp->vc = vc;
			tp->ve = ve;
			tp->t1 = (ve - vs) / acc;
			tp->t2 = 0;
			tp->t3 = 0;
			tp->L1 = vs * tp->t1 + 0.5 * acc * tp->t1 * tp->t1;
			tp->L2 = 0;
			tp->L3 = 0;
			tp->t = tp->t1;
		}
		else if (ve < vf && vf < vs)//只有减速段
		{
			ve = sqrt(vs * vs + 2 * dec * L);
			vc = vs;
			tp->vs = vs;
			tp->vc = vs;
			tp->ve = ve;
			tp->t1 = 0;
			tp->t2 = 0;
			tp->t3 = (ve - vs) / dec;
			tp->L1 = 0;
			tp->L2 = 0;
			tp->L3 = vc * tp->t3 + 0.5 * dec * tp->t3 * tp->t3;
			tp->t = tp->t3;
		}
		else//存在加速段和减速段
		{
			vc = vf;
			tp->vs = vs;
			tp->vc = vc;
			tp->ve = ve;
			tp->t1 = (vc - vs) / acc;
			tp->t2 = 0;
			tp->t3 = (ve - vc) / dec;
			tp->L1 = vs * tp->t1 + 0.5 * acc * tp->t1 * tp->t1;
			tp->L2 = 0;
			tp->L3 = vc * tp->t3 + 0.5 * dec * tp->t3 * tp->t3;
			tp->t = tp->t1 + tp->t3;
		}
	}
	return 0;
}

/**
 * \brief 计算梯形加减速任意时刻的加速度.
 * 
 * \param tp	已规划好的梯形速度结构体。
 * \param t		相对于梯形曲线起点的时刻。
 * \return		对应时刻的加速度。
 */
double calcTrapezoidalAcc(TrapeProfile_t* tp, double t)
{
	if (t < 0.0)
		return 0.0;
	else if (t < tp->t1)
		return tp->acc;
	else if (t < tp->t1 + tp->t2)
		return 0.0;
	else
		return tp->dec;
}

/**
 * \brief 计算梯形加减速任意时刻的速度.
 *
 * \param tp	梯形速度曲线结构体。
 * \param t		相对于该段起点的时刻。
 * \return		对应时刻的位移。
 */
double  calcTrapezoidalVel(TrapeProfile_t* tp, double t)
{
	double vt = 0.0;
	if (tp->L < 1.0E-5)
	{
		vt = 0.0;
	}
	else if (t < tp->t1)
	{
		vt = tp->vs + tp->acc * t;
	}
	else if (t < tp->t1 + tp->t2)
	{
		vt = tp->vc;
	}
	else
	{
		vt = tp->vc + tp->dec * (t - tp->t1 - tp->t2);
	}
	return vt;
}

/**
 * \brief 计算梯形加减速任意时刻的位移.
 *
 * \param tp	梯形速度曲线结构体。
 * \param t		相对于该段起点的时刻。
 * \return		对应时刻的位移。
 */
double  calcTrapezoidalDist(TrapeProfile_t* tp, double t)
{
	double tmp = 0.0, Lt = 0.0;
	if (tp->L < 1.0E-5)
	{
		Lt = 0.0;
	}
	else if (t < tp->t1)
	{
		Lt = tp->vs * t + 0.5 * tp->acc * t * t;
	}
	else if (t < tp->t1 + tp->t2)
	{
		Lt = tp->L1 + tp->vc * (t - tp->t1);
	}
	else
	{
		tmp = t - tp->t1 - tp->t2;
		Lt = tp->L1 + tp->L2 + tp->vc * tmp + 0.5 * tp->dec * tmp * tmp;
	}
	return Lt;
}


/**
 * \brief 梯形时间同步.
 * 
 * \param tp
 * \param ts
 * \param L
 * \param vs
 * \param ve
 * \param acc
 * \param dec
 * \return 
 */
int calcTrapeProfileTimeSync(TrapeProfile_t* tp, double ts, double L, double vs, double ve, double amax, double dmax)
{
	double T1 = 0.0, T2 = 0.0, A = 0.0, B = 0.0, C = 0.0,vc=0.0, tmp = 0.0;
	double a, d,t1,t2,t3,L1,L2,L3,La,Ld;
	if (ts < 1.0e-5)return 1;
	// 计算T1，T2
	if (vs < ve)
	{
		if (ve < 1.0e-5)
		{
			T1 = DBL_MAX;
			T2 = DBL_MAX;
		}
		else if (vs < 1.0e-5)
		{
			T1 = (ve - vs) / amax + (L - (ve * ve - vs * vs) / (2.0 * amax)) / ve;
			T2 = DBL_MAX;
		}

		else
		{
			T1 = (ve - vs) / amax + (L - (ve * ve - vs * vs) / (2.0 * amax)) / ve;
			T2 = (ve - vs) / (dmax)+(L - (ve * ve - vs * vs) / (2.0 * dmax)) / vs;
		}
	}
	else
	{
		// vs >= ve
		if (vs < 1.0e-5)
		{
			T1 = DBL_MAX;
			T2 = DBL_MAX;
		}
		else if (ve < 1.0e-5)
		{
			T1 = (ve - vs) / (-dmax) + (L - (ve * ve - vs * vs) / (-2.0 * dmax)) / vs;
			T2 = DBL_MAX;
		}
		else
		{
			T1 = (ve - vs) / (-dmax) + (L - (ve * ve - vs * vs) / (-2.0 * dmax)) / vs;
			T2 = (ve - vs) / (-amax) + (L - (ve * ve - vs * vs) / (-2.0 * amax)) / ve;
		}
	}
	if (ts < T1)//加速度段，匀速段，减速段
	{
		a = amax;
		d = -dmax;
		A = a - d;
		B = 2.0 * a * d * ts + 2.0 * d * vs - 2.0 * a * ve;
		C = a * ve * ve - d * vs * vs - 2.0 * a * d * L;
		tmp = B * B - 4.0 * A * C;
		if (fabs(A) > 1.0e-3)
			vc = (-B - sqrt(fabs(tmp))) / (2.0 * A);
		else
			vc = -C / B;
		vc = (-B-sqrt(fabs(tmp)))/ (2.0 * A);
		t1 = (vc - vs) / a;
		t3 = (ve - vc) / d;
		t2 = ts - t1 - t3;
		L1 = vs * t1 + 0.5 * amax * t1 * t1;
		L3 = vc * t3 - 0.5 * dmax * t3 * t3;
		L2 = L - L1 - L3;
	}
	else if (ts < T2)//加速度段，匀速段
	{
		if (vs < ve)
		{
			//加速，匀速，加速
			a = amax;
			d = amax;
			A = a - d;
			B = 2.0 * a * d * ts + 2.0 * d * vs - 2.0 * a * ve;
			C = a * ve * ve - d * vs * vs - 2.0 * a * d * L;
			tmp = B * B - 4.0 * A * C;
			if (abs(A) > 1.0e-3)
				vc = (-B - sqrt(fabs(tmp))) / (2.0 * A);
			else
				vc = -C / B;
			t1 = (vc - vs) / a;
			t3 = (ve - vc) / d;
			t2 = ts - t1 - t3;
			L1 = vs * t1 + 0.5 * a * t1 * t1;
			L2 = vc * t2;
			L3 = vc * t3 + 0.5 * d * t3 * t3;
		}
		else
		{
			//减速，匀速，减速
			a = -amax;
			d = -dmax;
			A = a - d;
			B = 2.0 * a * d * ts + 2.0 * d * vs - 2.0 * a * ve;
			C = a * ve * ve - d * vs * vs - 2.0 * a * d * L;
			if (abs(A) > 1.0e-3)

				vc = (-B - sqrt(abs(B * B - 4 * A * C))) / (2.0 * A);
			else
				vc = -C / B;
			t1 = (vc - vs) / a;
			t3 = (ve - vc) / d;
			t2 = ts - t1 - t3;
			L1 = vs * t1 + 0.5 * a * t1 * t1;
			L2 = vc * t2;
			L3 = vc * t3 + 0.5 * d * t3 * t3;
		}
	}
	else
	{	//ts>T2
		// 减速，匀速，加速
		a = -amax;
		d = dmax;
		A = a - d;
		B = 2.0 * a * d * ts + 2.0 * d * vs - 2.0 * a * ve;
		C = a * ve * ve - d * vs * vs - 2 * a * d * L;
		tmp = B * B - 4.0 * A * C;
		if (tmp >= 0.0 && L > vs * vs / (2.0 * amax) + ve * ve / (2.0 * dmax))
		{
			//有大于0的解
			La = vs * vs / (2.0 * amax);
			Ld = ve * ve / (2.0 * dmax);		
			if (abs(A) > 1.0e-3)
				vc = (-B - sqrt(tmp)) / (2 * A);
			else
				vc = -C / B;
			t1 = (vc - vs) / a;
			t3 = (ve - vc) / d;
			t2 = ts - t1 - t3;
			L1 = vs * t1 + 0.5 * a * t1 * t1;
			L2 = vc * t2;
			L3 = vc * t3 + 0.5 * d * t3 * t3;
		}
		else
		La = vs * vs / (2 * amax);
		Ld = ve * ve / (2 * dmax);
		//无大于零的解，需调整速度边界使方程有解
		if (L > vs * vs / (2.0 * amax))
			//可调整末速度使方程有解
			ve = sqrt(2 * dmax * (L - vs * vs / (2.0 * amax)));
		else
			//只能降低起步速度，匀速处理
		vs = L / ts;
		ve = vs;
		B = 2 * a * d * ts + 2 * d * vs - 2 * a * ve;
		C = a * ve * ve - d * vs * vs - 2 * a * d * L;
		tmp = B * B - 4.0 * A * C;
		if (abs(A) > 1.0e-3)
			vc = (-B - sqrt(tmp)) / (2 * A);
		else
			vc = -C / B;
		t1 = (vc - vs) / a;
		t3 = (ve - vc) / d;
		t2 = ts - t1 - t3;
		L1 = vs * t1 + 0.5 * a * t1 * t1;
		L2 = vc * t2;
		L3 = vc * t3 + 0.5 * d * t3 * t3;
	}
	return 0;
}
