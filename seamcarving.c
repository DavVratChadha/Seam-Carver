#include "seamcarving.h"
#include "c_img.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void calc_energy(struct rgb_img *im, struct rgb_img **grad){

    double rx, ry, gx, gy, bx, by;
    double width = im->width;
    double height = im->height;

    create_img(grad, height, width);

    for(int y = 0; y < height; y++){
        for(int x = 0; x < width; x++){
            if(x == 0){
                rx = pow(get_pixel(im, y, 1, 0) - get_pixel(im, y, width-1, 0), 2);
                
                gx = pow(get_pixel(im, y, 1, 1) - get_pixel(im, y, width-1, 1), 2);
                
                bx = pow(get_pixel(im, y, 1, 2) - get_pixel(im, y, width-1, 2), 2);
            }

            else if(x == width-1){
                rx = pow(get_pixel(im, y, 0, 0) - get_pixel(im, y, width-2, 0), 2);
                
                gx = pow(get_pixel(im, y, 0, 1) - get_pixel(im, y, width-2, 1), 2);
                
                bx = pow(get_pixel(im, y, 0, 2) - get_pixel(im, y, width-2, 2), 2);

            }

            else{
                rx = pow(get_pixel(im, y, x+1, 0) - get_pixel(im, y, x-1, 0), 2);
                
                gx = pow(get_pixel(im, y, x+1, 1) - get_pixel(im, y, x-1, 1), 2);
                
                bx = pow(get_pixel(im, y, x+1, 2) - get_pixel(im, y, x-1, 2), 2);
            }

            if(y == 0){
                ry = pow(get_pixel(im, 1, x, 0) - get_pixel(im, height-1, x, 0), 2);
                
                gy = pow(get_pixel(im, 1, x, 1) - get_pixel(im, height-1, x, 1), 2);
                
                by = pow(get_pixel(im, 1, x, 2) - get_pixel(im, height-1, x, 2), 2);
            }

            else if(y == height-1){
                ry = pow(get_pixel(im, 0, x, 0) - get_pixel(im, height-2, x, 0), 2);
                
                gy = pow(get_pixel(im, 0, x, 1) - get_pixel(im, height-2, x, 1), 2);
                
                by = pow(get_pixel(im, 0, x, 2) - get_pixel(im, height-2, x, 2), 2);
            }

            else{
                ry = pow(get_pixel(im, y+1, x, 0) - get_pixel(im, y-1, x, 0), 2);
                
                gy = pow(get_pixel(im, y+1, x, 1) - get_pixel(im, y-1, x, 1), 2);
                
                by = pow(get_pixel(im, y+1, x, 2) - get_pixel(im, y-1, x, 2), 2);
            }
            
            double delta_x = rx + gx + bx;
            double delta_y = ry + gy + by;
            double final = (uint8_t)(sqrt(delta_x + delta_y)/10);
            
            set_pixel(*grad , y , x, final, final, final);
        }
    }
};

double minimum(double a, double b, double c){
    if(a <= b && a <= c){
        return a;
    }
    else if(b <= a && b <= c){
        return b;
    }
    else{
        return c;
    }
}


void dynamic_seam(struct rgb_img *grad, double **best_arr){
    int height = grad->height;
    int width = grad->width;
    *best_arr = (double *)malloc(sizeof(double)*width*height);
    double *prev_vals = (double *)malloc(sizeof(double)*3*height);
    
    
    for(int ind = 0; ind < width; ind++){
        (*best_arr)[ind] = (double)get_pixel(grad, 0, ind, 0);
    }
    for(int i = 1; i < height; i++){
        for(int j = 0; j < width; j++){
            double prev_vals[3];
            double min_val;
            //gets value above it
            prev_vals[0] = (*best_arr)[(i-1)*width + j];

            //gets value left
            if(j >= 1){
                 prev_vals[1] = (*best_arr)[(i-1)*width + j - 1];
            }else{
                prev_vals[1] = 100000000;
            }

            //gets value right
            if(j < width - 1){
              prev_vals[2] = (*best_arr)[(i-1)*width + j + 1];
              
            }else{
                prev_vals[2] = 1000000000;
            }

            //find min
            min_val = minimum(prev_vals[0],prev_vals[1],prev_vals[2]);
            
            
            (*best_arr)[i*width + j] = min_val;
            (*best_arr)[i*width + j] += (double)get_pixel(grad, i, j, 0);
        }
    }
}

int min_index(double *best, int width, int i, int j){
    if(j > 0 && j < width - 1){
        double val1 = *(best + width*i + j - 1);
        double val2 = *(best + width*i + j);
        double val3 = *(best + width*i + j + 1);

        if(val1 < val2 && val1 < val3){
            return j - 1;
        }
        else if(val2 < val1 && val2 < val3){
            return j;
        }
        else{
            return j + 1;
        }
    }

    else if(j == 0){
        double val1 = (*best + width*i + j);
        double val2 = (*best + width*i + j + 1);
        if(val1 < val2){
            return j;
        }
        else{
            return j + 1;
        }
    }
    else if(j == width - 1){
        double val1 = (*best + width*i + j - 1);
        double val2 = (*best + width*i + j);
        if(val1 < val2){
            return j - 1;
        }
        else{
            return j;
        }
    }
    return 0;
}


void recover_path(double *best, int height, int width, int **path){
    *path = (int *)malloc(sizeof(int)*height);
    int min_ind;
    int min = 10000000;
    //min of last row
    for(int j = 0; j < width; j++){
       // printf("%f",best[(height-1)*(width) + j]);
        if(best[(height-1)*(width) + j] < min){
            min = best[(height-1)*(width) + j];
            min_ind = j;
        }
    }
    *(*path + height - 1) = min_ind;

    int j;  
    for(int i = height - 2; i > -1; i--){
        j = min_ind; 
        
        min_ind = min_index(best, width, i, j);
        *(*path + i) = min_ind;
        
    }
    
    return;
}


void remove_seam(struct rgb_img *src, struct rgb_img **dest, int *path){
    double width = src->width;
    double height = src->height;

    create_img(dest, height, width - 1);

    for(int i = 0; i < height; i++){
        for(int j = 0; j < width - 1; j++){
            if(j != *(path + i)){
                set_pixel(*dest , i , j, get_pixel(src, i, j, 0), get_pixel(src, i, j, 1), get_pixel(src, i, j, 2));
            }
        }
    }

}
