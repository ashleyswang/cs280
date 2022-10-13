// DO NOT EDIT REST OF THE FILE
// ONLY WRITE CODE BELOW THE "WRITE BELOW" COMMENTS

#include<cmath>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<iostream>

int main() {
    // You are required to create the matrices for 
    // the mentioned questions and print them to 
    // std::cout

    // Format I/O to print exact floats
    Eigen::IOFormat format(5);

    // Question 1
    /* 
    The transform matrix [[ 0 1 -2]  
                          [-1 0  2]
                          [ 0 0  1]] rotates the point 270 degrees (3pi/2 rads)
    about the origin (counterclockwise) and shifts it 2 leftwards and 2 upwards.

    Upper left 2x2 matrix [[0 1], [-1 0]] has the form of a rotation matrix with
    theta = 270 degrees. The third column acts at the translation, translating
    x by -2 (or 2 left) and y by 2 (or 2 up).
    */

    // Transform Matrix
    /*
    Eigen::Matrix3f transform;
    transform <<  0, 1, -2,
                 -1, 0,  2,
                  0, 0,  1;
    std::cout << "Transform Matrix: \n" << transform << std::endl;
    */
    
    std::cout << "------------------------\n";

    //---------------------------------------------------------------------
    // Question 2
    std::cout << "Question 2\n------------------------" << std::endl;
    // matrix definition for Question 2
    Eigen::Matrix3f rot_45;

    // Write the 3×3 transformation matrix for a 45 degrees
    // clockwise rotation in 2D (assuming homogeneous coordinates) 
    // Use the << operator to create the rot_45 matrix (WRITE BELOW)
    // DONT PRINT TO stdout
    
    /* 
    45 degree clockwise rotation = -45 degree rotation about the origin
    cos(-45 degrees) = 1/sqrt(2)    sin(-45 degrees) = -1/sqrt(2)
    */

    rot_45 <<  1/sqrt(2), 1/sqrt(2),  0, 
              -1/sqrt(2), 1/sqrt(2),  0, 
                       0,         0,  1;
    std::cout << rot_45 << std::endl;
    std::cout << "------------------------\n";
    
    //---------------------------------------------------------------------
    // Question 3
    std::cout << "Question 3\n------------------------" << std::endl;

    // matrix definition for Question 3
    Eigen::Matrix4f translation;

    // Write the 4×4 transform matrix to move a point by (-2,8,3)
    // Use the << operator to create the translation matrix (WRITE BELOW)
    
    /*
    No scaling and/or rotation, upper 3x3 is identity matrix.
    Fourth column is translation vector with 1 at the end.
    */

    translation << 1, 0, 0, -2,
                   0, 1, 0,  8,
                   0, 0, 1,  3, 
                   0, 0, 0,  1; 
    std::cout << translation << std::endl;
    std::cout << "------------------------\n";

    //---------------------------------------------------------------------
    // Question 4
    std::cout << "Question 4\n------------------------" << std::endl;
    
    // Define and create (using << operator) the matrices here. 
    // Make sure that they have the correct dimensions and ordering.
    Eigen::Vector3f bottom_left(1,5,1);
    Eigen::Vector3f upper_right(4,8,1);
    // Print the initial transformation matrices and the final transformation 
    // matrix to std out below

    // Move point (1,5) to the origin
    std::cout << "4a: 'Translate to the Origin' Transformation Matrix:" << std::endl;
    Eigen::Matrix3f translate_to_origin;
    translate_to_origin <<  1,  0,  -1,
                            0,  1,  -5,
                            0,  0,   1;
    std::cout << translate_to_origin << std::endl;
    
    // Scale the rectangle to be the same size as the target rectangle
    std::cout << "\n4b: 'Scale to Target Rectangle Size' Transformation Matrix:" << std::endl;
    Eigen::Matrix3f scale;
    scale <<  2,      0,  0,      // horizontal scale by 2
              0,  1/3.0,  0,      // vertical scale by 1/3
              0,      0,  1;
    std::cout << scale.format(format) << std::endl;
    
    // Move back points to new position
    std::cout << "\n4c: 'Translate to New Target Position' Transformation Matrix:" << std::endl;
    Eigen::Matrix3f translate_to_target_pos;
    translate_to_target_pos <<  1,  0,  2,      // move right by 2    
                                0,  1,  0,      // no vertical translation
                                0,  0,  1;
    std::cout << translate_to_target_pos << std::endl;

    std::cout << "\n4d: Multiplication Result of 3 Transformation Matrices:" << std::endl;
    Eigen::Matrix3f final_transform = translate_to_target_pos * scale * translate_to_origin;
    std::cout << final_transform << std::endl << std::endl;

    bottom_left = final_transform * bottom_left;
    upper_right = final_transform * upper_right;
    std::cout << "final bottom left: (" << bottom_left[0] << "," << bottom_left[1] << ")" << std::endl;
    std::cout << "final upper right: (" << upper_right[0] << "," << upper_right[1] << ")" << std::endl;

    return 0;
}
