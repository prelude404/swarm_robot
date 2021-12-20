#include <Hungary.h>

bool Hungary::get_matrix(Eigen::VectorXd cur_x, Eigen::VectorXd cur_y, Eigen::VectorXd target_x,Eigen::VectorXd target_y)
{
    double mean_x = cur_x.mean();
    double mean_y = cur_y.mean();
    double mean_x_form = target_x.mean();
    double mean_y_form = target_y.mean();

    assign = new int[num_swarm];
    for (int i = 0; i < num_swarm; i++){
        assign[i] = -1;
    }
    mat = new double *[num_swarm];
    for (int i = 0; i < num_swarm; i++){
        mat[i] = new double[num_swarm];
    }
    matRcd = new double *[num_swarm];
    for (int i = 0; i < num_swarm; i++){
        matRcd[i] = new double[num_swarm];
    }
    for(int i=0;i<num_swarm;i++){
		for(int j=0;j<num_swarm;j++){
            mat[i][j] = sqrt(pow(cur_x(i) - mean_x - target_x(j) + mean_x_form, 2) + pow(cur_y(i) - mean_y - target_y(j) + mean_y_form, 2));
            matRcd[i][j] = mat[i][j];
        }
    }
    totalCost = 0;
    return true;
}

void Hungary::rowSub()
{
	double *minEmt=new double[num_swarm];for(int i=0;i<num_swarm;i++)minEmt[i]=(double)1e8;
	for(int i=0;i<num_swarm;i++)for(int j=0;j<num_swarm;j++)if(mat[i][j]<minEmt[i])minEmt[i]=mat[i][j];
	for(int i=0;i<num_swarm;i++)for(int j=0;j<num_swarm;j++)mat[i][j]-=minEmt[i];
	delete []minEmt;
}

void Hungary::columnSub()
{
	double *minEmt=new double[num_swarm];for(int j=0;j<num_swarm;j++)minEmt[j]=(double)1e8;
	for(int j=0;j<num_swarm;j++)for(int i=0;i<num_swarm;i++)if(mat[i][j]<minEmt[j])minEmt[j]=mat[i][j];
	for(int j=0;j<num_swarm;j++)for(int i=0;i<num_swarm;i++)mat[i][j]-=minEmt[j];
	delete []minEmt;
}

bool Hungary::isOptimal(int *assign)
{

	int *tAssign=new int[num_swarm];for(int i=0;i<num_swarm;i++)tAssign[i]=-1;
	int *nZero=new int[num_swarm];
	bool *rowIsUsed=new bool[num_swarm];
	bool *columnIsUsed=new bool[num_swarm];
	for(int i=0;i<num_swarm;i++)rowIsUsed[i]=columnIsUsed[i]=0;

	int nLine=0;
	while(nLine<num_swarm){
		for(int i=0;i<num_swarm;i++)nZero[i]=0;
		for(int i=0;i<num_swarm;i++){
			if(rowIsUsed[i]==1)continue;
			for(int j=0;j<num_swarm;j++){
				if(columnIsUsed[j]!=1&&mat[i][j]==0.0)nZero[i]++;
			}
		}

		int minZeros=num_swarm;
		int rowId=-1;
		for(int i=0;i<num_swarm;i++){
			if(rowIsUsed[i]==0&&nZero[i]<minZeros&&nZero[i]>0){
				minZeros=nZero[i];
				rowId=i;
			}
		}
		if(rowId==-1)break;
		for(int j=0;j<num_swarm;j++){
			if(mat[rowId][j]==0.0&&columnIsUsed[j]==0){
				rowIsUsed[rowId]=1;
				columnIsUsed[j]=1;
				tAssign[rowId]=j;
				break;
			}
		}
		nLine++;
	}
	for(int i=0;i<num_swarm;i++)assign[i]=tAssign[i];
	delete []tAssign;
	delete []nZero;
	delete []rowIsUsed;
	delete []columnIsUsed;

	for(int i=0;i<num_swarm;i++)if(assign[i]==-1)return false;
	return true;
}

void Hungary::matTrans()
{
	bool *rowTip=new bool[num_swarm];
	bool *columnTip=new bool[num_swarm];
	bool *rowLine=new bool[num_swarm];
	bool *columnLine=new bool[num_swarm];
	for(int i=0;i<num_swarm;i++)rowTip[i]=columnTip[i]=rowLine[i]=columnLine[i]=0;

	//打勾
	for(int i=0;i<num_swarm;i++)if(assign[i]==-1)rowTip[i]=1;

	while(1){
		int preTip=0;
		for(int i=0;i<num_swarm;i++)preTip=preTip+rowTip[i]+columnTip[i];
		for(int i=0;i<num_swarm;i++){
			if(rowTip[i]==1){
				for(int j=0;j<num_swarm;j++){
					if(mat[i][j]==0.0)columnTip[j]=1;
				}
			}
		}
		for(int j=0;j<num_swarm;j++){
			if(columnTip[j]==1){
				for(int i=0;i<num_swarm;i++){
					if(assign[i]==j)rowTip[i]=1;
				}
			}
		}
		int curTip=0;
		for(int i=0;i<num_swarm;i++)curTip=curTip+rowTip[i]+columnTip[i];
		if(preTip==curTip)break;
	}
	
	//画线
	for(int i=0;i<num_swarm;i++){
		if(rowTip[i]==0)rowLine[i]=1;
		if(columnTip[i]==1)columnLine[i]=1;
	}

	//找最小元素
	double minElmt=(double)1e8;
	for(int i=0;i<num_swarm;i++)for(int j=0;j<num_swarm;j++)if(rowLine[i]==0&&columnLine[j]==0&&mat[i][j]<minElmt)minElmt=mat[i][j];
	//变换
	for(int i=0;i<num_swarm;i++)if(rowTip[i]==1)for(int j=0;j<num_swarm;j++)mat[i][j]-=minElmt;
	for(int j=0;j<num_swarm;j++)if(columnTip[j]==1)for(int i=0;i<num_swarm;i++)mat[i][j]+=minElmt;

	delete []rowTip;
	delete []columnTip;
	delete []rowLine;
	delete []columnLine;
}

void Hungary::hungary(Eigen::VectorXd cur_x, Eigen::VectorXd cur_y, Eigen::VectorXd target_x, Eigen::VectorXd target_y)
{
	get_matrix(cur_x, cur_y, target_x, target_y);
    rowSub();   //行归约
    columnSub();//列归约

	//如果不能找到n个独立的0元素，则对矩阵进行变换
	while(!isOptimal(assign)){
		matTrans();
	}

	for(int i=0;i<num_swarm;i++)totalCost+=matRcd[i][assign[i]];
	for(int i=0;i<num_swarm;i++)delete []mat[i];delete []mat;
	for(int i=0;i<num_swarm;i++)delete []matRcd[i];delete []matRcd;
}