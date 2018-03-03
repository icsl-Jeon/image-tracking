#ifndef BOXOPERATOR_H
#define BOXOPERATOR_H


#include <iostream>
#include <vector>

// box proposal functions
#define areaRationbtw12 0.2
#define maxOverlapRatio 0.7

struct Box{

    double upper_right_x;
    double upper_right_y;
    double lower_left_x;
    double lower_left_y;
    int area;
    Box(): upper_right_x(0),upper_right_y(0),lower_left_x(0),lower_left_y(0),area(0){}
};

//for sort purpose, we define like inverse!!!!1
inline bool operator <(const Box &a, const Box &b){
    return a.area > b.area;
}
typedef std::vector< std::vector<bool> > boolMatrix; // bool matrix 
typedef std::vector< std::vector<int> > intMatrix;

//for index space
inline int getIntersection(Box box1,Box box2)
{


    //index
    int intsectArea=0;
    if(! ((box1.upper_right_y<box2.lower_left_y) ||
            (box1.lower_left_y>box2.upper_right_y)||
            (box1.lower_left_x < box2.upper_right_x)||
            (box1.upper_right_x>box2.lower_left_x)))
       {
        intsectArea=(std::min(box1.upper_right_y,box2.upper_right_y)-std::max(box1.lower_left_y,box2.lower_left_y)+1)*
                (-std::max(box1.upper_right_x,box2.upper_right_x)+std::min(box1.lower_left_x,box2.lower_left_x)+1);

        }
    return intsectArea;

}


//for real space
inline int getIntersection2(Box box1,Box box2)
{


    //index
    int intsectArea=0;
    if(! ((box1.upper_right_x<box2.lower_left_x) ||
            (box1.lower_left_y>box2.upper_right_y)||
            (box1.lower_left_x > box2.upper_right_x)||
            (box1.upper_right_x<box2.lower_left_x)))
       {
        intsectArea=(std::min(box1.upper_right_x,box2.upper_right_x)-std::max(box1.lower_left_x,box2.lower_left_x)+1)*
                (std::min(box1.upper_right_y,box2.upper_right_y)-std::max(box1.lower_left_y,box2.lower_left_y)+1);

        }
    return intsectArea;

}

inline std::vector<Box> getMaxArea(std::vector<int> hist, int n)
{

        // Create an empty stack. The stack holds indexes of hist[] array
        // The bars stored in stack are always in increasing order of their
        // heights.
        std::vector<int> s;

        int max_area = 0; // Initalize max area
        Box max_Box;

        std::vector<Box> box_buffer;
        Box init_Box;
        box_buffer.reserve(5);
        box_buffer.push_back(init_Box);
        box_buffer.push_back(init_Box);
        box_buffer.push_back(init_Box);
        box_buffer.push_back(init_Box);
        box_buffer.push_back(init_Box);


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

                //compare every cell with current
                for(int buffer_idx=0;buffer_idx<5;buffer_idx++)
                    if(area_with_top>box_buffer[buffer_idx].area)
                    {

                        //box construct!
                        Box winning_box;
                        winning_box.area=area_with_top;
                        if(!s.empty())
                        {
                            winning_box.upper_right_x=i-1;
                            winning_box.upper_right_y=hist[tp]-1;

                            winning_box.lower_left_x=s.back()+1;
                            winning_box.lower_left_y=0;
                        }
                        else
                        {
                            winning_box.upper_right_x=i-1;
                            winning_box.upper_right_y=hist[tp]-1;

                            winning_box.lower_left_x=0;
                            winning_box.lower_left_y=0;

                        }

                        for(int erase_idx=4;erase_idx>buffer_idx;erase_idx--)
                            box_buffer[erase_idx]=box_buffer[erase_idx-1];
                        box_buffer[buffer_idx]=winning_box;
                        break;

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


        //compare every cell with current
        for(int buffer_idx=0;buffer_idx<5;buffer_idx++)
            if(area_with_top>box_buffer[buffer_idx].area)
            {

                //box construct!
                Box winning_box;
                winning_box.area=area_with_top;
                if(!s.empty())
                {
                    winning_box.upper_right_x=i-1;
                    winning_box.upper_right_y=hist[tp]-1;

                    winning_box.lower_left_x=s.back()+1;
                    winning_box.lower_left_y=0;
                }
                else
                {
                    winning_box.upper_right_x=i-1;
                    winning_box.upper_right_y=hist[tp]-1;

                    winning_box.lower_left_x=0;
                    winning_box.lower_left_y=0;

                }

                for(int erase_idx=4;erase_idx>buffer_idx;erase_idx--)
                    box_buffer[erase_idx]=box_buffer[erase_idx-1];
                box_buffer[buffer_idx]=winning_box;
                break;

            }


    }


    Box reallyKeptbox;
    //keep only the most valuable second biggest

    for(int i=1;i<5;i++)
    {

        /**
        std::cout<<"----------------------------top"<<i+1<<"-----------------"<<std::endl;
        std::cout<<"max area"<<std::endl;
        std::cout<<"upper: ["<<box_buffer[0].upper_right_x<<" , "<<box_buffer[0].upper_right_y<<"]"<<std::endl;
        std::cout<<"lower: ["<<box_buffer[0].lower_left_x<<" , "<<box_buffer[0].lower_left_y<<"]"<<std::endl;
        std::cout<<"\n"<<std::endl;


        std::cout<<"next max area"<<std::endl;
        std::cout<<"upper: ["<<box_buffer[i].upper_right_x<<" , "<<box_buffer[i].upper_right_y<<"]"<<std::endl;
        std::cout<<"lower: ["<<box_buffer[i].lower_left_x<<" , "<<box_buffer[i].lower_left_y<<"]"<<std::endl;
        std::cout<<"\n"<<std::endl;


        std::cout<<"max area"<<box_buffer[0].area<<" this area"<<box_buffer[i].area<<std::endl;
        std::cout<<"overlap area: "<<getIntersection2(box_buffer[0],box_buffer[i])<<std::endl;


        std::cout<<"1st cond: "<<(box_buffer[0].area*areaRationbtw12<box_buffer[i].area)<<std::endl;
        std::cout<<"2nd cond: "<<(getIntersection2(box_buffer[0],box_buffer[i])<box_buffer[0].area*maxOverlapRatio)<<std::endl;
        std::cout<<"3rd cond: "<<(getIntersection2(box_buffer[0],box_buffer[i])<box_buffer[i].area*maxOverlapRatio)<<std::endl;

        **/

        if((box_buffer[0].area*areaRationbtw12<box_buffer[i].area) &&
          (getIntersection2(box_buffer[0],box_buffer[i])<box_buffer[0].area*maxOverlapRatio)&&
            (getIntersection2(box_buffer[0],box_buffer[i])<box_buffer[i].area*maxOverlapRatio)) //comparable and little overlap
        {

            // Caution!! here Box represent real x y space.
            reallyKeptbox=box_buffer[i]; //if that condition is satisfied.
            std::vector<Box> returnBox;
            returnBox.push_back(box_buffer[0]);
            returnBox.push_back(reallyKeptbox);
            return returnBox;
        }
    }
    /**
    std::cout<<"---------------------"<<std::endl;
    std::cout<<"max histogram:"<<std::endl;
    std::cout<<"upper: ["<<box_buffer[0].upper_right_x<<" , "<<box_buffer[0].upper_right_y<<"]"<<std::endl;
    std::cout<<"lower: ["<<box_buffer[0].lower_left_x<<" , "<<box_buffer[0].lower_left_y<<"]"<<std::endl;
    std::cout<<"\n"<<std::endl;
    std::cout<<"max area"<<box_buffer[0].area<<std::endl;
    **/
    std::vector<Box> returnBox;
    returnBox.push_back(box_buffer[0]);
    return returnBox;
};


// Returns area of the largest rectangle with all 1s in A[][]
// TODO : how next max ?
inline std::vector<Box> maxRectangle(intMatrix A)
{

        int R=A.size(); int C=A[0].size();
        // Calculate area for first row and initialize it as
        // result
        std::vector<Box> this_Boxes = getMaxArea(A[0],C);

        std::vector<Box> box_buffer;
        box_buffer.reserve(5); // only 1st and 2nd biggest
        Box init_box;
        //buffer initialization

        box_buffer.push_back(init_box);
        box_buffer.push_back(init_box);
        box_buffer.push_back(init_box);
        box_buffer.push_back(init_box);
        box_buffer.push_back(init_box);

        int insert_idx=0;
        for(std::vector<Box>::iterator it=this_Boxes.begin();it!=this_Boxes.end();it++)
           {
                it->lower_left_y=it->lower_left_x; it->lower_left_x=0;
                int tmp=it->upper_right_y;
                it->upper_right_y=it->upper_right_x; it->upper_right_x=0-tmp;
                box_buffer[insert_idx++]=*it;
           }



        // iterate over row to find maximum rectangular area
        // considering each row as histogram
        for (int i = 1; i < R; i++)
        {

                for (int j = 0; j < C; j++)
                        // if A[i][j] is 1 then add A[i -1][j]
                        if (A[i][j]) A[i][j] += A[i - 1][j];



                        std::vector<Box> this_Boxes_rowwise=getMaxArea(A[i],C);

                        /**
                        cout << "upper: [ " << this_Box.upper_right_x<<" , "<<this_Box.upper_right_y<<" ]"<<  endl;
                        cout << "lower: [ " << this_Box.lower_left_x<<" , "<<this_Box.lower_left_y<<" ]"  <<  endl;
                        cout <<"area: "<<this_Box.area<<endl;
                        **/
                        for(std::vector<Box>::iterator it=this_Boxes_rowwise.begin();it!=this_Boxes_rowwise.end();it++)
                            for(int k=0;k<5;k++)
                               if(it->area>box_buffer[k].area) //take out the throne!
                               {
                                   //index conversion
                                   it->lower_left_y=it->lower_left_x; it->lower_left_x=i;
                                   int tmp=it->upper_right_y;
                                   it->upper_right_y=it->upper_right_x; it->upper_right_x=i-tmp;
                                   //
                                   for(int l=4;l>k;l--)
                                       box_buffer[l]=box_buffer[l-1];
                                   box_buffer[k]=*it;
                                   break;

                               }

        }

        // we will return one biggest box or two biggest box that satisfy those relationship

        Box reallyKeptbox;

        for(int i=1;i<5;i++)
        {
            /**
            std::cout<<"----------------------------top"<<i+1<<"-----------------"<<std::endl;
            std::cout<<"max area"<<std::endl;
            std::cout<<"upper: ["<<box_buffer[0].upper_right_x<<" , "<<box_buffer[0].upper_right_y<<"]"<<std::endl;
            std::cout<<"lower: ["<<box_buffer[0].lower_left_x<<" , "<<box_buffer[0].lower_left_y<<"]"<<std::endl;
            std::cout<<"\n"<<std::endl;


            std::cout<<"next max area"<<std::endl;
            std::cout<<"upper: ["<<box_buffer[i].upper_right_x<<" , "<<box_buffer[i].upper_right_y<<"]"<<std::endl;
            std::cout<<"lower: ["<<box_buffer[i].lower_left_x<<" , "<<box_buffer[i].lower_left_y<<"]"<<std::endl;
            std::cout<<"\n"<<std::endl;


            std::cout<<"max area"<<box_buffer[0].area<<" this area"<<box_buffer[i].area<<std::endl;
            std::cout<<"overlap area: "<<getIntersection(box_buffer[0],box_buffer[i])<<std::endl;


            std::cout<<"1st cond: "<<(box_buffer[0].area*areaRationbtw12<box_buffer[i].area)<<std::endl;
            std::cout<<"2nd cond: "<<(getIntersection(box_buffer[0],box_buffer[i])<box_buffer[0].area*maxOverlapRatio)<<std::endl;
            std::cout<<"3rd cond: "<<(getIntersection(box_buffer[0],box_buffer[i])<box_buffer[i].area*maxOverlapRatio)<<std::endl;
            **/
            if((box_buffer[0].area*areaRationbtw12<box_buffer[i].area) &&
                  (getIntersection(box_buffer[0],box_buffer[i])<box_buffer[0].area*maxOverlapRatio)&&
                    (getIntersection(box_buffer[0],box_buffer[i])<box_buffer[i].area*maxOverlapRatio)) //comparable and little overlap
            {
                reallyKeptbox=box_buffer[i]; //if that condition is satisfied
                box_buffer[1]=reallyKeptbox;
                box_buffer.erase(box_buffer.begin()+2,box_buffer.end());
                return box_buffer;
            }


        }

        box_buffer.erase(box_buffer.begin()+1,box_buffer.end());
        return box_buffer;
};



#endif // BOXOPERATOR_H

