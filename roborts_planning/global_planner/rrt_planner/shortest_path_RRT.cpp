#include<bits/stdc++.h>
#define eps 1e-5
#define ll long long
inline ll read(){
    char c=getchar();while (c!='-'&&(c<'0'||c>'9'))c=getchar();
    ll k=0,kk=1;if (c=='-')c=getchar(),kk=-1;
    while (c>='0'&&c<='9')k=k*10+c-'0',c=getchar();return kk*k;
}using namespace std;
void write(ll x){if (x<0)x=-x,putchar('-');if (x/10)write(x/10);putchar(x%10+'0');}
void writeln(ll x){write(x);puts("");}
const int N=110,M=210;
const int rand_max=2147483647;
const double pi=acos(-1);
const double p=0.5;// has 50% chance by random and 50% chnce goto the nearest point
const int l_pace=30,r_pace=50;//the rage of max_pace_length is 10
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine gen(seed);
std::normal_distribution<double> dis(0,pi/2);
map<pair<int,int>,int>flag,point;
int n,m,x,y,xx,yy,d,ans;
int d1,d2,last1[2010],last2[2010];
char s[N][M];
pair<int,int>z[2010],z1[2010],z2[2010];
void get_map(){
    n=read();m=read();
    for (int i=1;i<=n;i++)scanf("%s",&s[i][1]);
    x=read();y=read();xx=read();yy=read();
}
/*
void navagation(double x,double y){
    printf("next_point_is: %.10lf %.10lf\b");
}
*/
pair<int,int> check(int x,int y,double xx,double yy,double pace){
    //the nearest available point
    //direct is x,y->xx,yy; delta_length is pace
    double dx=xx-x,dy=yy-y,dxy=sqrt(dx*dx+dy*dy);
    xx=x+pace/dxy*dx;yy=y+pace/dxy*dy;
    dx=(xx-x)/20,dy=(yy-y)/20;
    pair<int,int> d=make_pair(x,y);
    for (double i=x,j=y;fabs(i-xx)>eps&&fabs(j-yy)>eps;i+=dx,j+=dy){
        int ii=i+0.5,jj=j+0.5;
        if (i<1||j<1||i>n||j>m)break;
        if (s[ii][jj]!='0')break;
        d=make_pair(ii,jj);
    }return d;
}
int L2(pair<int,int> x,pair<int,int> y){
    return (x.first-y.first)*(x.first-y.first)+(x.second-y.second)*(x.second-y.second);
}
pair<int,int> next_point(pair<int,int> now,pair<int,int> last,pair<int,int> z[],int d){
    double pace=rand()%(r_pace-l_pace+1)+l_pace;
    if (rand()>rand_max*p){//the nearest way to z[]
        int dd=1,L2_dd=L2(now,z[1]);
        for (int i=2;i<=d;i++){
            int k=L2(now,z[i]);
            if (k<L2_dd){
                L2_dd=k;dd=i;
            }
        }if (pace*pace>L2_dd)pace=sqrt(L2_dd);
        return check(now.first,now.second,z[dd].first,z[dd].second,pace);
    }else{
        double dx=now.first-last.first,dy=now.second-last.second,dxy=sqrt(dx*dx+dy*dy);
        double cos_now=dx/dxy,sin_now=dy/dxy;
        double theta=fmod(dis(gen),pi),cos_theta=cos(theta),sin_theta=sin(theta);
        double cos_new = cos_now*cos_theta - sin_now*sin_theta;
        double sin_new = sin_now*cos_theta + cos_now*sin_theta;
        return check(now.first,now.second,now.first+cos_new,now.second+sin_new,pace);
    }
}
void dfs1(int x){
    if (x!=1)dfs1(last1[x]);
    z[++d]=z1[x];
}
void dfs2(int x){
    z[++d]=z2[x];
    if (x!=1)dfs2(last2[x]);
}
bool Bidirectional_RRT(){
    srand(1);
    z1[d1=1]=make_pair(x,y);flag[z1[1]]=1;point[z1[1]]=1;
    z2[d2=1]=make_pair(xx,yy);flag[z2[1]]=2;point[z2[2]]=1;
    for (int l=1;l<=1000;l++){
        ans+=l;
        if (l&1){
            int dd=rand()%d1+1;
            last1[++d1]=dd;
            z1[d1]=next_point(z1[dd],z1[last1[dd]],z2,d2);

            if (flag[z1[d1]]==1){
                d1--;continue;
            }
            if (flag[z1[d1]]==2){//get the same point
                dfs1(last1[d1]);dfs2(point[z1[d1]]);
                return 1;
            }flag[z1[d1]]=1;point[z1[d1]]=d1;
            //   cout<<z1[d1].first<<' '<<z1[d1].second<<' '<<1<<endl;
        }else{
            int dd=rand()%d2+1;
            last2[++d2]=dd;
            z2[d2]=next_point(z2[dd],z2[last2[dd]],z1,d1);

            if (flag[z2[d2]]==2){
                d2--;continue;
            }
            if (flag[z2[d2]]==1){//get the same point
                dfs1(point[z2[d2]]);dfs2(last2[d2]);
                return 1;
            }flag[z2[d2]]=2;point[z2[d2]]=d2;
            //  cout<<z2[d2].first<<' '<<z2[d2].second<<' '<<2<<endl;
        }
    }return 0;
}
void output(){
    for (int i=1;i<=d;i++)write(z[i].first),putchar(' '),writeln(z[i].second);
}
signed main(){
    freopen("/home/cogito/CLionProjects/ICRA_shortest_path/map.in","r",stdin);
    get_map();
    while (!Bidirectional_RRT())1;
    output();
    cout<<ans<<endl;
}
//
// Created by cogito on 2021/1/4.
//

