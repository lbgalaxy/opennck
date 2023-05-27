/*****************************************************************//**
 * \file   testFunction.c
 * \brief  测试用例的实现，后续算法的测试用例也写在该文件。
 * 
 * \author galaxy
 * \date   March 2023
 *********************************************************************/

#include"publicData.h"

/**
 * \brief 测试梯形加减速.
 * 
 */
void test_calcTrapezoidalProfile(void)
{
	TrapeProfile_t tp;
	FILE *fp = fopen("tpProfile.txt", "w");
	if(fp==NULL)
	{
		printf("fopen error!\n");
		return;
	}
	double L = 0.1, vs = 10, vmax = 50, ve = 20, amax = 500, dmax = 400;//给定参数
	double	ti=0.0,Li,vi,ai,dt=0.001;
	int ret = calcTrapezoidalProfile(L, vs, vmax, ve, amax, dmax, &tp);//执行梯形速度规划
	if (ret)
	{
		printf("calcTrapezoidalProfile error ret=%d\n ",ret);
		goto err;
	}
	//计算梯形速度曲线的离散数据，进行绘图可视化
	while (1)
	{
		if (ti > tp.t)
		{
			ti = tp.t;
			Li = calcTrapezoidalDist(&tp, ti);
			vi = calcTrapezoidalVel(&tp, ti);
			ai = calcTrapezoidalAcc(&tp, ti);
			fprintf(fp, "%lf %lf %lf %lf\n", ti, Li, vi, ai);
			break;
		}
		Li=calcTrapezoidalDist(&tp, ti);
		vi = calcTrapezoidalVel(&tp, ti);
		ai = calcTrapezoidalAcc(&tp, ti);
		fprintf(fp, "%lf %lf %lf %lf\n", ti, Li, vi,ai);
		ti = ti + 0.001;//周期为0.001s
	}
err:
	fclose(fp);
	return;
}

int main(void)
{
	test_calcTrapezoidalProfile();
	return 0;
}
