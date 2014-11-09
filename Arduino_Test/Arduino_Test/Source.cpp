#include <iostream>

using namespace std;

void stuff(double &y, double v);

int main(){
	double v = 5;
	double x = 2;
	
	stuff(x,v);
	stuff(x, v);
	cout << x << endl;

	return(0);
}

void stuff(double &y, double v){
	y = y * v;

}