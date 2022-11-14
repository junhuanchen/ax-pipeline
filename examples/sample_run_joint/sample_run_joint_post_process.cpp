/*
 * AXERA is pleased to support the open source community by making ax-samples available.
 *
 * Copyright (c) 2022, AXERA Semiconductor (Shanghai) Co., Ltd. All rights reserved.
 *
 * Licensed under the BSD 3-Clause License (the "License"); you may not use this file except
 * in compliance with the License. You may obtain a copy of the License at
 *
 * https://opensource.org/licenses/BSD-3-Clause
 *
 * Unless required by applicable law or agreed to in writing, software distributed
 * under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 */

/*
 * Author: ZHEQIUSHUI
 */

#include <vector>
#include <string.h>
#include "fstream"

#include "sample_run_joint.h"

#include "joint.h"
#include "../utilities/sample_log.h"

#include "../common/sample_def.h"

#include "opencv2/opencv.hpp"

#include "../sample_vin_ivps_joint_vo/sample_vin_ivps_joint_vo.h"

#include "omp.h"
int sample_parse_yolov5_param(char *json_file_path)
{
    return 0;
}

// cv::Mat mask_result_2(480, 854, CV_8UC4);
// cv::Mat mask_result_2(480, 854, CV_8UC4);
// extern pthread_mutex_t g_result_mutex;
// pthread_mutex_t *get_g_result_mutex();

// extern "C" unsigned char *get_disp_data()
// {
//     return mask_result_2.data;
// }

/// @brief 模型后处理函数
/// @param nOutputSize 输出的节点数
/// @param pOutputsInfo 输出的节点对应的信息，包含维度信息、节点名称等
/// @param pOutputs 输出的节点的数据指针，包含物理地址、虚拟地址等
/// @param pResults 目标检测的结果信息
/// @param SAMPLE_ALGO_WIDTH 算法的输入宽
/// @param SAMPLE_ALGO_HEIGHT 算法的输入高
/// @param SAMPLE_MAJOR_STREAM_WIDTH 相机图像的宽
/// @param SAMPLE_MAJOR_STREAM_HEIGHT 相机图像的高
void sample_run_joint_post_process(AX_U32 nOutputSize, AX_JOINT_IOMETA_T *pOutputsInfo, AX_JOINT_IO_BUFFER_T *pOutputs, sample_run_joint_results *pResults,
                                   int SAMPLE_ALGO_WIDTH, int SAMPLE_ALGO_HEIGHT, int SAMPLE_MAJOR_STREAM_WIDTH, int SAMPLE_MAJOR_STREAM_HEIGHT)
{

    #include "time.h"
    #define CALC_FPS(tips)                                                                                         \
        {                                                                                                          \
            static int fcnt = 0;                                                                                   \
            fcnt++;                                                                                                \
            static struct timespec ts1, ts2;                                                                       \
            clock_gettime(CLOCK_MONOTONIC, &ts2);                                                                  \
            if ((ts2.tv_sec * 1000 + ts2.tv_nsec / 1000000) - (ts1.tv_sec * 1000 + ts1.tv_nsec / 1000000) >= 1000) \
            {                                                                                                      \
                printf("%s => H26X FPS:%d     \r\n", tips, fcnt);                                                  \
                ts1 = ts2;                                                                                         \
                fcnt = 0;                                                                                          \
            }                                                                                                      \
        }

    CALC_FPS("sample_run_joint_post_process");

    static cv::Mat output_mask(192, 192, CV_8UC1, cv::Scalar(0));

    auto &output = pOutputsInfo[0];
    auto &info = pOutputs[0];
    auto ptr = ((float *)info.pVirAddr);
    auto pixel_num = 192 * 192;
    for (int j = 0; j != pixel_num; ++j)
    {
        output_mask.data[j] = (uint8_t)(ptr[j] < ptr[j + pixel_num]) ? 0 : 255;
    }

    // quick
    static cv::Mat tmp(SAMPLE_MINOR_STREAM_HEIGHT, SAMPLE_MINOR_STREAM_WIDTH, CV_8UC1);
    cv::resize(output_mask, tmp, cv::Size(SAMPLE_MINOR_STREAM_WIDTH, SAMPLE_MINOR_STREAM_HEIGHT), 0, 0, cv::INTER_NEAREST);
    // static cv::Mat mask_result(SAMPLE_MINOR_STREAM_HEIGHT, SAMPLE_MINOR_STREAM_WIDTH, CV_8UC4);
    // cv::cvtColor(tmp, mask_result, cv::COLOR_GRAY2RGBA);

    // use mask
    static cv::Mat mask_img = cv::imread("./mask.png", cv::IMREAD_UNCHANGED);

    static cv::Mat mask_result(SAMPLE_MINOR_STREAM_HEIGHT, SAMPLE_MINOR_STREAM_WIDTH, CV_8UC4);
    memset(mask_result.data, 0, mask_result.size().width * mask_result.size().height * mask_result.channels());

    mask_img.copyTo(mask_result, tmp);

    extern AX_U32 OSD_Grp[SAMPLE_REGION_COUNT];

    RGN_GROUP_CFG_T tRgnGroupConfig[SAMPLE_REGION_COUNT] = {
        {OSD_Grp[0], 0x11, SAMPLE_MINOR_STREAM_WIDTH, SAMPLE_MINOR_STREAM_HEIGHT, 1, AX_IVPS_RGN_LAYER_COVER},
    };

    AX_S32 ret = 0;
    AX_IVPS_RGN_DISP_GROUP_S tDisp;

    // memcpy(abgr_data, get_disp_data(),SAMPLE_MINOR_STREAM_WIDTH*SAMPLE_MINOR_STREAM_HEIGHT*4);

    memset(&tDisp, 0, sizeof(AX_IVPS_RGN_DISP_GROUP_S));

    tDisp.nNum = tRgnGroupConfig[0].nRgnNum;
    tDisp.tChnAttr.nAlpha = 1024;
    tDisp.tChnAttr.eFormat = AX_FORMAT_ARGB8888;
    tDisp.tChnAttr.nZindex = 1;
    tDisp.tChnAttr.nBitColor.nColor = 0xFF0000;
    tDisp.tChnAttr.nBitColor.bEnable = AX_FALSE;
    tDisp.tChnAttr.nBitColor.nColorInv = 0xFF;
    tDisp.tChnAttr.nBitColor.nColorInvThr = 0xA0A0A0;

    tDisp.arrDisp[0].bShow = AX_TRUE;
    tDisp.arrDisp[0].eType = AX_IVPS_RGN_TYPE_OSD;

    tDisp.arrDisp[0].uDisp.tOSD.bEnable = AX_TRUE;
    tDisp.arrDisp[0].uDisp.tOSD.enRgbFormat = AX_FORMAT_ARGB8888;
    tDisp.arrDisp[0].uDisp.tOSD.u32Zindex = 1;
    tDisp.arrDisp[0].uDisp.tOSD.u32ColorKey = 0x0;
    tDisp.arrDisp[0].uDisp.tOSD.u32BgColorLo = 0xFFFFFFFF;
    tDisp.arrDisp[0].uDisp.tOSD.u32BgColorHi = 0xFFFFFFFF;
    tDisp.arrDisp[0].uDisp.tOSD.u32BmpWidth = tRgnGroupConfig[0].nChnWidth;
    tDisp.arrDisp[0].uDisp.tOSD.u32BmpHeight = tRgnGroupConfig[0].nChnHeight;
    tDisp.arrDisp[0].uDisp.tOSD.u32DstXoffset = 0;
    tDisp.arrDisp[0].uDisp.tOSD.u32DstYoffset = 32;
    tDisp.arrDisp[0].uDisp.tOSD.u64PhyAddr = 0;
    tDisp.arrDisp[0].uDisp.tOSD.pBitmap = mask_result.data;

    ret = AX_IVPS_RGN_Update(g_arrRgnThreadParam->hChnRgn, &tDisp);
    if (0 != ret)
    {
        ALOGE("[%d][0x%02x] AX_IVPS_RGN_Update fail, ret=0x%x, hChnRgn=%d", g_arrRgnThreadParam->nGroupIdx, g_arrRgnThreadParam->nFilter, ret, g_arrRgnThreadParam->hChnRgn);
    }

}