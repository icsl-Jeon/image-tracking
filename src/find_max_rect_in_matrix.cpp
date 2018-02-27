// C++ program to find maximum rectangular area in linear time
#include<iostream>
#include <vector>

using namespace std;

struct box{
    int upper_right_x;
    int upper_right_y;
    int lower_left_x;
    int lower_left_y;
    int area;
	box(): upper_right_x(0),upper_right_y(0),lower_left_x(0),lower_left_y(0),area(0){}
};

// The main function to find the maximum rectangular area under given
// histogram with n bars
box getMaxArea(vector<int> hist, int n)
{

        // Create an empty stack. The stack holds indexes of hist[] array
        // The bars stored in stack are always in increasing order of their
        // heights.
        vector<int> s;

        int max_area = 0; // Initalize max area
        box max_box;
        int tp; // To store top of stack
        int area_with_top; // To store area with top bar as the smallest bar

        // Run through all bars of given histogram
        int i = 0;
        while (i < n)
        {


			// If this bar is higher than the bar on top stack, push it to stack
			if (s.empty() || hist[s.back()] <= hist[i])
					s.push_back(i++);

			// If this bar is lower than top of stack, then calculate area of rectangle
			// with stack top as the smallest (or minimum height) bar. 'i' is
			// 'right index' for the top and element before top in stack is 'left index'
			else
			{
				tp = s.back(); // store the top index
				s.pop_back(); // pop the top

				// Calculate the area with hist[tp] stack as smallest bar
				area_with_top = hist[tp] * (s.empty() ? i : i - s.back() - 1);

				// update max area, if needed
				if (max_area < area_with_top)
					{
						if(!s.empty())
						{
							max_box.upper_right_x=i-1;
							max_box.upper_right_y=hist[tp]-1;

							max_box.lower_left_x=s.back()+1;
							max_box.lower_left_y=0;
						}
						else
						{
							max_box.upper_right_x=i-1;
							max_box.upper_right_y=hist[tp]-1;

							max_box.lower_left_x=0;
							max_box.lower_left_y=0;

						}

						max_area = area_with_top;
						max_box.area=max_area;

					}
			}
		}



	// Now pop the remaining bars from stack and calculate area with every
	// popped bar as the smallest bar
	while (s.empty() == false)
	{
		tp = s.back();
		s.pop_back();
		area_with_top = hist[tp] * (s.empty() ? i : i - s.back() - 1);

		if (max_area < area_with_top)
			{



				if(!s.empty())
				{
					max_box.upper_right_x=i-1;
					max_box.upper_right_y=hist[tp]-1;

					max_box.lower_left_x=s.back()+1;
					max_box.lower_left_y=0;
				}
				else
				{
					max_box.upper_right_x=i-1;
					max_box.upper_right_y=hist[tp]-1;

					max_box.lower_left_x=0;
					max_box.lower_left_y=0;

				}


				max_area = area_with_top;
				max_box.area=max_area;

			}
	}

	return max_box;
};


// Returns area of the largest rectangle with all 1s in A[][]
box maxRectangle(vector< vector<int> >  A)
{

        int R=A.size(); int C=A[0].size();
        cout<<R<<C<<endl;
        // Calculate area for first row and initialize it as
        // result
        box max_box = getMaxArea(A[0],C);


        // iterate over row to find maximum rectangular area
        // considering each row as histogram
        for (int i = 1; i < R; i++)
        {

                for (int j = 0; j < C; j++)
                        // if A[i][j] is 1 then add A[i -1][j]
                        if (A[i][j]) A[i][j] += A[i - 1][j];


                // Update result if area with current row (as last row)
                // of rectangle) is more

                        box this_box=getMaxArea(A[i],C);

                        cout << "upper: [ " << this_box.upper_right_x<<" , "<<this_box.upper_right_y<<" ]"<<  endl;
                        cout << "lower: [ " << this_box.lower_left_x<<" , "<<this_box.lower_left_y<<" ]"  <<  endl;
                        cout <<"area: "<<this_box.area<<endl;


                // change index: matrix index of upper right corner and lower left corner

                        if (this_box.area>=max_box.area)

                        {
                                this_box.lower_left_y=this_box.lower_left_x; this_box.lower_left_x=i;
                                int tmp=this_box.upper_right_y;
                                this_box.upper_right_y=this_box.upper_right_x; this_box.upper_right_x=i-tmp;

                                max_box=this_box;
                        }


        }

        return max_box;
}



// max hist main code
/**
int main()
{
        int hist_array[] = {2, 2, 3, 3};
        vector<int> hist(hist_array,hist_array+sizeof(hist_array)/sizeof(int));
        box max_box=getMaxArea(hist, 4);


        //here, some problem exist. but trivial.
        cout << "upper: [ " << max_box.upper_right_x<<" , "<<max_box.upper_right_y<<" ]"<<  endl;
        cout << "lower: [ " << max_box.lower_left_x<<" , "<<max_box.lower_left_y<<" ]"  <<  endl;
        cout <<"area: "<<max_box.area<<endl;

        return 0;
}
**/


int main()
{

        int mat_array[4][4] = { {0, 1, 1, 0},
                                   {1, 1, 1, 1},
                                   {1, 1, 1, 1},
                                   {1, 1, 0, 0},
                                 };
        vector< vector<int> > mat;


        for(int i=0;i<4;i++)
        {

            vector<int> row_array;
            for(int j=0;j<4;j++)
                    row_array.push_back(mat_array[i][j]);

            mat.push_back(row_array);

        }

        cout<<mat.size()<<endl;

        box max_box=maxRectangle(mat);



        cout << "upper: [ " << max_box.upper_right_x<<" , "<<max_box.upper_right_y<<" ]"<<  endl;
        cout << "lower: [ " << max_box.lower_left_x<<" , "<<max_box.lower_left_y<<" ]"  <<  endl;
        cout <<"area: "<<max_box.area<<endl;


        return 0;
}


