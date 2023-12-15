#include <iostream>
#include <cmath>
#include <vector>
#include "ap_int.h"
#include "Filters_Weights.h"

using namespace std;

float max(float a, float b, float c, float d)
{
    if (a > b && a > c && a > d)
        return a;

    else if (b > c && b > d)
        return b;

    else if (c > d)
        return c;

    return d;
}

void maxPooling(float arr[], int w, int h, float out[]) {
    int t = 0;
	#pragma HLS array_partition variable=arr type=complete dim=1
    for (int i = 0; i < h - 1; i += 2) {
		#pragma HLS unroll
        for (int j = 0; j < w - 1; j += 2) {
            // Check if the next pooling window goes beyond the width or height
            int width_limit = (j + 1 < w - 1) ? j + 1 : w - 1;
            int height_limit = (i + 1 < h - 1) ? i + 1 : h - 1;

            out[t++] = max(
                arr[i * w + j], arr[(i + 1) * w + j],
                arr[i * w + width_limit], arr[height_limit * w + width_limit]
            );
        }
    }
}

void convOperation(float arr[], float filter[], int w, int h, float out[], float bias)
{
    int t = 0;
    float sum;
    for (int i = 1; i < h - 1; i++)
    {
        for (int j = 1; j < w - 1; j++)
        {
            sum = bias;
            for (int m = 0; m < 3; m++)
            {
                for (int n = 0; n < 3; n++)
                {
                    sum += arr[(i - 1 + m) * w + (j - 1 + n)] * filter[m * 3 + n];
                }
            }
            out[t++] = sum;
        }
    }
}

void fullyConnected(float in[], int size_in, const float weights[], int size_w, float out[], float bias[])
{
    float sum;
    float w = 0;
    float inn = 0;
    for (int i = 0; i < size_w; i++)
    {
		#pragma HLS unroll
        sum = 0;
        for (int j = 0; j < size_in; j++)
        {
			#pragma HLS unroll
            sum += weights[(size_w * j) + i] * in[j];
        }
        out[i] = sum + bias[i];
    }
    return;
}

void relu(float in[], float out[], int size) 
{
    for (int i = 0; i < size; i++) 
    {
		#pragma HLS unroll factor=2
    	out[i] = (in[i] >= 0) ? in[i] : 0;
    }
}

void softmax(float scores[], int size) 
{
    float max_score = scores[0];
    for (int i = 1; i < size; ++i) 
    {
        if (scores[i] > max_score) 
        {
            max_score = scores[i];
        }
    }

    float sum = 0.0;
    for (int i = 0; i < size; ++i) 
    {
        scores[i] = exp(scores[i] - max_score);
        sum += scores[i];
    }

    for (int i = 0; i < size; ++i) 
    {
        scores[i] /= sum;
    }
}

int inference(int in[1200])
{
	// --------------------------- For interfacing with userspace --------------------------------------
	#pragma HLS INTERFACE s_axilite port=return
	#pragma HLS INTERFACE s_axilite port=in

    // --------------------------------- Initializing CNN ----------------------------------------------
    const int height0 = 30;
    const int width0 = 40;
    const int height0_conv = height0 - 2;
    const int width0_conv = width0 - 2;
    const int height1 = height0_conv / 2;
    const int width1 = width0_conv / 2;
    const int height1_conv = height1 - 2;
    const int width1_conv = width1 - 2;
    const int height2 = height1_conv / 2;
    const int width2 = width1_conv / 2;
    const int height2_conv = height2 - 2;
    const int width2_conv = width2 - 2;
    const int height_last = height2_conv / 2;
    const int width_last = width2_conv / 2;

    float conv_out[height0 * width0] = {};
    float pool_out[height0 * width0] = {};

    // --------------------------------- Convolutional Layers ------------------------------------------
    for (int i = 0; i < height0 * width0; i++)
    {
		#pragma HLS unroll
        pool_out[i] = in[i] / 255.0;
    }


    convOperation(pool_out, layer_0, width0, height0, conv_out, bias_conv_0);
    relu(conv_out, conv_out, width0 * height0);
    maxPooling(conv_out, width0_conv, height0_conv, pool_out);

    convOperation(pool_out, layer_1, width1, height1, conv_out, bias_conv_1);
    relu(conv_out, conv_out, width1 * height1);
    maxPooling(conv_out, width1_conv, height1_conv, pool_out);

    convOperation(pool_out, layer_2, width2, height2, conv_out, bias_conv_2);
    relu(conv_out, conv_out, width2 * height2);
    maxPooling(conv_out, width2_conv, height2_conv, pool_out);


    // --------------------------------- Fully-Connected Layer -----------------------------------------
    int weights_0_size = 64;
    int weights_1_size = 32;
    int weights_2_size = 12;
    int weights_3_size = 4;

    fullyConnected(pool_out, width_last * height_last, weights_0, weights_0_size, conv_out, bias_layer0);
    relu(conv_out, conv_out, weights_0_size);

    fullyConnected(conv_out, weights_0_size, weights_1, weights_1_size, pool_out, bias_layer1);
    relu(pool_out, pool_out, weights_1_size);

    fullyConnected(pool_out, weights_1_size, weights_2, weights_2_size, conv_out, bias_layer2);
    relu(conv_out, conv_out, weights_2_size);

    fullyConnected(conv_out, weights_2_size, weights_3, weights_3_size, pool_out, bias_layer3);

    // --------------------------- Soft-max the results and output this --------------------------------
    softmax(pool_out, weights_3_size);

    for(int i = 0; i < weights_3_size; i++){
    	if (round(pool_out[i]) == 1){
    		return i;
    	}
    }

    return 0;
}






