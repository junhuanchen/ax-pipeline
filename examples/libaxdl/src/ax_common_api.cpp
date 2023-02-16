#include "ax_common_api.h"

#include "c_api.h"
#include "utilities/mat_pixel_affine.h"
#include "../../utilities/sample_log.h"

#include "string.h"

#include "npu_cv_kit/ax_npu_imgproc.h"
#include "ax_sys_api.h"

int ax_sys_memalloc(unsigned long long int *phyaddr, void **pviraddr, unsigned int size, unsigned int align, const char *token)
{
    return AX_SYS_MemAlloc(phyaddr, pviraddr, size, align, (const AX_S8 *)token);
}

int ax_sys_memfree(unsigned long long int phyaddr, void *pviraddr)
{
    return AX_SYS_MemFree(phyaddr, pviraddr);
}

void cvt(axdl_image_t *src, AX_NPU_CV_Image *dst)
{
    memset(dst, 0, sizeof(AX_NPU_CV_Image));
    dst->pPhy = src->pPhy;
    dst->pVir = (unsigned char *)src->pVir;
    dst->nHeight = src->nHeight;
    dst->nWidth = src->nWidth;
    dst->nSize = src->nSize;
    dst->tStride.nW = src->tStride_C;
    switch (src->eDtype)
    {
    case axdl_color_space_nv12:
        dst->eDtype = AX_NPU_CV_FDT_NV12;
        break;
    case axdl_color_space_nv21:
        dst->eDtype = AX_NPU_CV_FDT_NV21;
        break;
    case axdl_color_space_bgr:
        dst->eDtype = AX_NPU_CV_FDT_BGR;
        break;
    case axdl_color_space_rgb:
        dst->eDtype = AX_NPU_CV_FDT_RGB;
        break;
    default:
        dst->eDtype = AX_NPU_CV_FDT_UNKNOWN;
        break;
    }
}

int ax_npu_csc(axdl_image_t *src, axdl_image_t *dst)
{
    AX_NPU_CV_Image npu_src, npu_dst;
    cvt(src, &npu_src);
    cvt(dst, &npu_dst);
    return AX_NPU_CV_CSC(AX_NPU_MODEL_TYPE_1_1_1, &npu_src, &npu_dst);
}

int ax_npu_warp(axdl_image_t *src, axdl_image_t *dst, const float *pMat33, const int const_val)
{
    AX_NPU_CV_Image npu_src, npu_dst;
    cvt(src, &npu_src);
    cvt(dst, &npu_dst);
    return AX_NPU_CV_Warp(AX_NPU_MODEL_TYPE_1_1_2, &npu_src, &npu_dst, pMat33, AX_NPU_CV_BILINEAR, const_val);
}

int ax_align_face(axdl_object_t *obj, axdl_image_t *src, axdl_image_t *dst)
{
    static float target[10] = {38.2946, 51.6963,
                               73.5318, 51.5014,
                               56.0252, 71.7366,
                               41.5493, 92.3655,
                               70.7299, 92.2041};
    float _tmp[10] = {obj->landmark[0].x, obj->landmark[0].y,
                      obj->landmark[1].x, obj->landmark[1].y,
                      obj->landmark[2].x, obj->landmark[2].y,
                      obj->landmark[3].x, obj->landmark[3].y,
                      obj->landmark[4].x, obj->landmark[4].y};
    float _m[6], _m_inv[6];
    get_affine_transform(_tmp, target, 5, _m);
    invert_affine_transform(_m, _m_inv);
    float mat3x3[3][3] = {
        {_m_inv[0], _m_inv[1], _m_inv[2]},
        {_m_inv[3], _m_inv[4], _m_inv[5]},
        {0, 0, 1}};

    dst->eDtype = src->eDtype;
    if (dst->eDtype == axdl_color_space_rgb || dst->eDtype == axdl_color_space_bgr)
    {
        dst->nSize = 112 * 112 * 3;
    }
    else if (dst->eDtype == axdl_color_space_nv12 || dst->eDtype == axdl_color_space_nv21)
    {
        dst->nSize = 112 * 112 * 1.5;
    }
    else
    {
        ALOGE("just only support BGR/RGB/NV12 format");
    }
    return ax_npu_warp(src, dst, &mat3x3[0][0], 128);
}