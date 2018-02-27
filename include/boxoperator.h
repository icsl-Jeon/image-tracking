#ifndef BOXOPERATOR_H
#define BOXOPERATOR_H


#include <iostream>
#include <vector>

// box proposal functions


struct Box{

    double upper_right_x;
    double upper_right_y;
    double lower_left_x;
    double lower_left_y;
    int area;
    Box(): upper_right_x(0),upper_right_y(0),lower_left_x(0),lower_left_y(0),area(0){}
};


typedef std::vector< std::vector<bool> > boolMatrix; // bool matrix 
typedef std::vector< std::vector<int> > intMatrix;


inline Box getMaxArea(std::vector<int> hist, int n)
{

        // Create an empty stack. The stack holds indexes of hist[] array
        // The bars stored in stack are always in increasing order of their
        // heights.
        std::vector<int> s;

        int max_area = 0; // Initalize max area
        Box max_Box;
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
                            max_Box.upper_right_x=i-1;
                            max_Box.upper_right_y=hist[tp]-1;

                            max_Box.lower_left_x=s.back()+1;
                            max_Box.lower_left_y=0;
                        }
                        else
                        {
                            max_Box.upper_right_x=i-1;
                            max_Box.upper_right_y=hist[tp]-1;

                            max_Box.lower_left_x=0;
                            max_Box.lower_left_y=0;

                        }

                        max_area = area_with_top;
                        max_Box.area=max_area;

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
                    max_Box.upper_right_x=i-1;
                    max_Box.upper_right_y=hist[tp]-1;

                    max_Box.lower_left_x=s.back()+1;
                    max_Box.lower_left_y=0;
                }
                else
                {
                    max_Box.upper_right_x=i-1;
                    max_Box.upper_right_y=hist[tp]-1;

                    max_Box.lower_left_x=0;
                    max_Box.lower_left_y=0;

                }


                max_area = area_with_top;
                max_Box.area=max_area;

            }
    }

    return max_Box;
};


// Returns area of the largest rectangle with all 1s in A[][]
// TODO : how next max ?
inline Box maxRectangle(intMatrix A)
{

        int R=A.size(); int C=A[0].size();
        // Calculate area for first row and initialize it as
        // result
        Box max_Box = getMaxArea(A[0],C);



        // iterate over row to find maximum rectangular area
        // considering each row as histogram
        for (int i = 1; i < R; i++)
        {

                for (int j = 0; j < C; j++)
                        // if A[i][j] is 1 then add A[i -1][j]
                        if (A[i][j]) A[i][j] += A[i - 1][j];


                // Update result if area with current row (as last row)
                // of rectangle) is more

                        Box this_Box=getMaxArea(A[i],C);
                        /**
                        cout << "upper: [ " << this_Box.upper_right_x<<" , "<<this_Box.upper_right_y<<" ]"<<  endl;
                        cout << "lower: [ " << this_Box.lower_left_x<<" , "<<this_Box.lower_left_y<<" ]"  <<  endl;
                        cout <<"area: "<<this_Box.area<<endl;
                        **/
                // change index: matrix index of upper right corner and lower left corner

                        if (this_Box.area>=max_Box.area)

                        {
                                this_Box.lower_left_y=this_Box.lower_left_x; this_Box.lower_left_x=i;
                                int tmp=this_Box.upper_right_y;
                                this_Box.upper_right_y=this_Box.upper_right_x; this_Box.upper_right_x=i-tmp;

                                max_Box=this_Box;
                        }


        }

        return max_Box;
};






#endif // BOXOPERATOR_H

