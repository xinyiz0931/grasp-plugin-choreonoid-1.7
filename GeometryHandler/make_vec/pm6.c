/*   �ݏ�@�ɂ���Ώ̍s��̌ŗL�l�v�Z   */
#include	<math.h>
#include	<stdlib.h>
#include	<stdio.h>
#define		NN		3
#define		SWAP(a,b)	{w=a; a=b; b=w;}
#define EPS         1E-20  /* ���e�덷 */
#define TINY        1E-20 /* 0 �ƌ��Ȃ��Ă悢�l */

/*   ���C���v���O�����̗�   */
/*main()
{
	int 	i,j,k,m,n;
	double	a[NN][NN],ev[NN],evec[NN][NN];
	datain(a,&n);
	printf("���߂�ŗL�l�̌� m= \n");
	scanf("%d",&m);
	pm(a,ev,evec,m,n);
}
*/


/*   ����   */
double sp(double x[],double y[],int n)
{
	int i;
	double s;
	s=0.0;
	for ( i=0 ; i<n ; ++i )
		s+=x[i]*y[i];
	return(s);
}

/*   ���K��   */
void
normal(double x[], int    n)
{
	int i;
	double s;
	s=sqrt(sp(x,x,n));
	for ( i=0 ; i<n ; ++i )
		x[i]=x[i]/s;
}

/*   �o���l�̐ݒ�   */
void
inival(double  x[],int	n)
{
	int i;
	for ( i=0 ; i<n ; ++i )
		x[i]=( rand() & 2047 )-4096;
	normal(x,n);
}

/*   �����x�N�g������ŗL�l���Z�o   */
void
rat(double x[],double y[],double *pmin,double *pmax,double *prq,int    n)
{
	int i;
	double q,rmin,rmax;
	double eps=1.0e-20,bignum=1.0e20;
	rmin=bignum;
	rmax=(-bignum);
	for ( i=0 ; i<n ; ++i )
	{
		if (fabs(y[i])<eps)
			rmin=eps;
		else
		{
			if (fabs(x[i])>eps)
			{
				q=y[i]/x[i];
				if (q<rmin) rmin=q;
				if (q>rmax) rmax=q;
			}
			else
				rmax=bignum;
		}
	}
	*prq=sp(x,y,n);
	*pmin=rmin;
	*pmax=rmax;
}


/*   �x�N�g���̓]��   */
void
copyv(double a[],double b[],int    n)
{
	int i;
	for ( i=0 ; i<n ; ++i )
		b[i]=a[i];
}

/*   �]�u�s��ƃx�N�g���̐�   */
void
mtxv(double a[NN][NN],double x[NN],double y[NN],int    n)
{
	int 	i,j;
	double	s;
	for ( i=0 ; i<n ; ++i )
	{
		s=0.0;
		for ( j=0 ; j<n ; ++j )
			s+=a[j][i]*x[j];
		y[i]=s;
	}
}

/*   �s��̓ǂݍ���   */
/*
void
datain(double a[NN][NN],int    *pn)
{
	int i,j,n;
	printf(" n= \n");
	scanf("%d",&n);
	*pn=n;
	for ( i=0 ; i<n ; ++i )
	{
		printf("%d �s�ڂ����Ă�������\n",i);
		for ( j=0 ; j<n ; ++j )
			scanf("%lf",&a[i][j]);
	}
}
*/



/*   �s�{�b�g�I��   */
void
pivot(double a[NN][NN],double b[],int    k,int n)
{
	int    i,j,imax;
	double g,aik,w;
	g=fabs(a[k][k]);
	imax=k;
	for ( i=k+1 ; i<n ; ++i )
	{
		aik=fabs(a[i][k]);
		if (aik>g)
		{
			g=aik;
			imax=i;
		}
	}
	if (imax==k) return;
	for ( j=k ; j<n ; ++j )
		SWAP(a[k][j],a[imax][j]);
	SWAP(b[k],b[imax]);
}

/*   ������������   */
void
gauss(double a[NN][NN],double b[NN],double x[NN],int    n)
{
	double p,q,s,eps=1.0e-20;
	int i,j,k;
	for ( k=0 ; k<n-1 ; ++k )
	{
		pivot(a,b,k,n);
		p=a[k][k];
		if (fabs(p)<eps) p=eps;
		for ( j=k ; j<n ; ++j )
			a[k][j]=a[k][j]/p;
		b[k]=b[k]/p;
		for ( i=k+1 ; i<n ; ++i )
		{
			q=a[i][k];
			for ( j=k ; j<n ; ++j )
				a[i][j]=a[i][j]-q*a[k][j];
			b[i]=b[i]-q*b[k];
		}
	}
	x[n-1]=b[n-1]/a[n-1][n-1];
	for ( k=n-2 ; k>=0 ; --k )
	{
		s=b[k];
		for ( j=k+1 ; j<n ; ++j )
			s=s-a[k][j]*x[j];
		x[k]=s;
	}
}




void
mout(double a[NN][NN], int n)
{
	int i,j;
	for ( i=0 ; i<n ; ++i )
	{
		for ( j=0 ; j<n ; ++j )
			printf("%12.7lf ",a[i][j]);
		printf("\n");
	}
}

void
vout(double x[NN],int n)
{
	int i;
	for ( i=0 ; i<n ; ++i )
		printf("%12.7lf  ",x[i]);
	printf("=v\n");
}

/*   �]�u�s��̌ŗL�x�N�g��   */
void
atev(
     double	a[NN][NN],  double	qev,double x[NN],
  int	n)
{
	int 	i,j,k,kmax=200;
	double 	y[NN],aa[NN][NN],b[NN]; //,xx[NN]
	double 	rmax,rmin,rrq;
	double 	eps=0.0001;
	/* �o���l��ݒ� */
	for ( i=0 ; i<n ; ++i )
	{
		for ( j=0 ; j<n ; ++j )
			aa[i][j]=a[j][i];
		aa[i][i]=a[i][i]-qev;
		b[i]=(-aa[i][n-1]);
	}
	gauss(aa,b,x,n-1);
	x[n-1]=1.0;
	normal(x,n);
	/* ���� */
	for ( k=0 ; k<kmax ; ++k )
	{
		/* �`������ */
		mtxv(a,x,y,n);
		/* �ŗL�l���v�Z */
		rat(x,y,&rmin,&rmax,&rrq,n);
		/* �������Ɉڂ� */
		copyv(y,x,n);
		/* �������P�ɐ��K�� */
		normal(x,n);
		/* �������� */
		if ((rmax-rmin)<=eps)
			break;
	}
}


/*   �ݏ�@   */
void
pm(
   double a0[NN][NN],double evv[NN],double vv[NN][NN],
   int	m,int n)
{
	int 	i,j,k,ban,kai,kmax=75;
	double 	x[NN],y[NN];//,z[NN];
	double 	rmax,rmin,rrq;
	double 	eps=EPS;
	double	ev,s,t;
	double	a[NN][NN],u[NN],v[NN];
	/* ���Ƃ̂`��ۑ� */
	for ( i=0 ; i<n ; ++i )
		for ( j=0 ; j<n ; ++j )
			a[i][j]=a0[i][j];
	/* �ŗL�l����������܂Ŕ��� */
	for ( ban=0 ; ban<m ; ++ban )
	{
		/* �o���l�����̐��� */
		for ( kai=0 ; kai<10 ; ++kai )
		{
			/* �o���l��ݒ� */
			inival(x,n);
			/* ���C�� ���[�v */
			for ( k=0 ; k<kmax ; ++k )
			{
				/* �`������  t�`������ */
				for ( i=0 ; i<n ; ++i )
				{
					s=0.0;
					for ( j=0 ; j<n ; ++j )
						s+=a[i][j]*x[j];
					y[i]=s;
				}
				/* �ŗL�l�v�Z (1) */
				rat(x,y,&rmin,&rmax,&rrq,n);
				/* �������� (1) */
				if ((rmax-rmin)<=eps)
					goto nagoya;
				/* �������Ɉڂ� */
				copyv(y,x,n);
				/* �������P�ɐ��K�� */
				normal(x,n);
			}
			//printf("�o���l����\n");
		} /* end of kai */
		/* ���������ꍇ */
nagoya:
		copyv(y,u,n);
		normal(u,n);
		copyv(u,v,n);
		atev(a,rrq,v,n);
		if(sp(u,v,n)<0.0)
			for ( i=0 ; i<n ; ++i )
				v[i]=(-v[i]);
		/* �ŗL�l�̐��x���� */
		s=t=0.0;
		for ( i=0 ; i<n ; ++i )
		{
			for ( j=0 ; j<n ; ++j )
				s+=v[i]*a0[i][j]*u[j];
			t+=u[i]*v[i];
		}
		ev=s/t;
		/* ���ʂ�z��ɏ������� */
		evv[ban]=ev;
		for ( i=0 ; i<n ; i++ )
			vv[ban][i]=-u[i];
			
			/* ���ʂ�\������ */
			/*		printf("�ŗL�l %17.8lf\n",ev);
			printf("�ŗL�x�N�g��\n");
			for ( i=0 ; i<n ; ++i ){
			printf("%15.8lf\n",u[i]);
			//			printf("%15.8lf\n",vv[ban][i]);
			}
			for ( i=0 ; i<3 ; ++i ){
			for ( j=0 ; j<3 ; ++j ){
			printf("%15.8lf\n",vv[i][j]);
			}
			}
			*/		/* �������� */
		for ( i=0 ; i<n ; ++i )
			for ( j=0 ; j<n ; ++j )
				a[i][j]=a[i][j]-ev*u[i]*v[j];
	} /* end of ban */
	return;
}
