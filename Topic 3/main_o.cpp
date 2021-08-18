// C++ program to demonstrate default behaviour of
// sort() in STL.
#include <vector>
#include <algorithm>
#include "iostream"
using namespace std;

int main()
{
//    int arr[] = { 1, 5, 8, 9, 6, 7, 3, 4, 2, 0 };
//    int n = sizeof(arr) / sizeof(arr[0]);
//
//    sort(arr,arr + n, greater<int>());
//
//    cout << "\nArray after sorting using "
//            "default sort is : \n";
//    for (auto &i : arr)
//        cout << arr[i] << " ";
    vector<int> A= {0,1,3,4,5,6,7,8,9,7};
    vector<int> Test (A.size(), 0);
    for (auto &i : Test){
        cout << i << endl;
    }
    sort(A.begin(),A.end());
    int a = INT32_MAX;
    cout << a;
    return 0;
}



