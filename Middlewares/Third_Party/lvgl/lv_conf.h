/**
 * @file lv_conf.h
 * Configuration file for v8.2.0
 */

/*
 * Copy this file as `lv_conf.h`
 * 1. simply next to the `lvgl` folder
 * 2. or any other places and
 *    - define `LV_CONF_INCLUDE_SIMPLE`
 *    - add the path as include path
 */

/* clang-format off */
#if 1 /*Set it to "1" to enable content*/

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*********************************************************************************

                                        ��ɫ����
                                        
 ***********************************************************************************/

/* ��ɫ���: 1(ÿ����1�ֽ�), 8(RGB332), 16(RGB565), 32(ARGB8888) */
#define LV_COLOR_DEPTH                      16

/* ����2�ֽڵ�RGB565��ɫ�������ʾ��8λ�ӿ�(����SPI) */
#define LV_COLOR_16_SWAP                    0


/* 1: ������Ļ͸��.
 * ��OSD���������ص���gui������.
 * Ҫ��' LV_COLOR_DEPTH = 32 '��ɫ����Ļ����ʽӦ�ñ��޸�: `style.body.opa = ...`*/
#define LV_COLOR_SCREEN_TRANSP              0

/* ������ɫ��Ϲ����������롣gpu���ܻ��Բ�ͬ�ķ�ʽ������ɫ��ϡ�
 * 0:ȡ����64:��x.75ȡ����128:��halfȡ����192:��x.25ȡ����254:��halfȡ�� */
#define LV_COLOR_MIX_ROUND_OFS              (LV_COLOR_DEPTH == 32 ? 0: 128)

/* ���ʹ��ɫ�ȼ������������������ɫ��ͼ������) */
#define LV_COLOR_CHROMA_KEY                 lv_color_hex(0x00ff00)         /* ���� */



/*********************************************************************************

                                        �ڴ�����
                                        
 ***********************************************************************************/

/* 0: ʹ�����õ� `lv_mem_alloc()` �� `lv_mem_free()`*/
#define LV_MEM_CUSTOM                       0
#if LV_MEM_CUSTOM == 0
    /* `lv_mem_alloc()`�ɻ�õ��ڴ��С(���ֽ�Ϊ��λ)(>= 2kB) */
    #define LV_MEM_SIZE                     (46U * 1024U)          /*[�ֽ�]*/

    /* Ϊ�ڴ������һ����ַ�������ǽ�����Ϊ��ͨ������䡣Ҳ�������ⲿSRAM�С� */
    #define LV_MEM_ADR                      0     /*0: δʹ��*/
    /* ���ڴ�����������ǵ�ַ�����������������LVGL���ڴ�ء�����my_malloc */
    #if LV_MEM_ADR == 0
        //#define LV_MEM_POOL_INCLUDE your_alloc_library  /* ���ʹ���ⲿ��������ȡ��ע�� */
        //#define LV_MEM_POOL_ALLOC   your_alloc          /* ���ʹ���ⲿ��������ȡ��ע�� */
    #endif

#else       /*LV_MEM_CUSTOM*/
    #define LV_MEM_CUSTOM_INCLUDE <stdlib.h>   /* ��̬�ڴ溯����ͷ */
    #define LV_MEM_CUSTOM_ALLOC   malloc
    #define LV_MEM_CUSTOM_FREE    free
    #define LV_MEM_CUSTOM_REALLOC realloc
#endif     /*LV_MEM_CUSTOM*/

/* ����Ⱦ�������ڲ����������ڼ�ʹ�õ��м��ڴ滺������������
 * ���û���㹻�Ļ���������ῴ��һ��������־��Ϣ. */
#define LV_MEM_BUF_MAX_NUM                  16

/* ʹ�ñ�׼�� `memcpy` �� `memset` ����LVGL�Լ��ĺ�����(���ܸ��죬Ҳ���ܲ������) */
#define LV_MEMCPY_MEMSET_STD                0



/*********************************************************************************

                                        HAL ����
                                        
 ***********************************************************************************/
 
/* Ĭ�ϵ���ʾˢ�����ڡ�LVGLʹ����������ػ��޸Ĺ������� */
#define LV_DISP_DEF_REFR_PERIOD             4      /*[ms]*/

/* �����豸�Ķ�ȡ����(�Ժ���Ϊ��λ) */
#define LV_INDEV_DEF_READ_PERIOD            4     /*[ms]*/

/* ʹ���Զ���tickԴ���Ժ���Ϊ��λ��������ʱ�䡣������Ҫ�ֶ����� `lv_tick_inc()` */
#define LV_TICK_CUSTOM                      0
#if LV_TICK_CUSTOM
    #define LV_TICK_CUSTOM_INCLUDE          "FreeRTOS.h"                /* ϵͳʱ�亯��ͷ */
    #define LV_TICK_CUSTOM_SYS_TIME_EXPR    (xTaskGetTickCount())       /* ����ϵͳ��ǰʱ��ı���ʽ(�Ժ���Ϊ��λ) */
#endif   /*LV_TICK_CUSTOM*/


/* Ĭ��ÿӢ��ĵ����������ڳ�ʼ��Ĭ�ϴ�С������С������С����ʽ��䡣
 * (���Ǻ���Ҫ������Ե��������޸�Ĭ�ϴ�С�Ϳո�) */
#define LV_DPI_DEF                          130     /*[px/inch]*/



/*********************************************************************************

                                        ����ѡ��
                                        
 ***********************************************************************************/
/*-------------
 * 1. ����
 *-----------*/


/* ���ø��ӵĻ�������
 * ��Ҫ������Ӱ���ݶȣ�Բ�ǣ�Բ������б�ߣ�ͼ��ת�����κ����� */
#define LV_DRAW_COMPLEX                     1
#if LV_DRAW_COMPLEX != 0

    /* ��������һЩ��Ӱ����
     * LV_SHADOW_CACHE_SIZEΪ���Ļ����С�������СΪ `��Ӱ���� + �뾶`
     * ������ LV_SHADOW_CACHE_SIZE^2 ���ڴ濪�� */
    #define LV_SHADOW_CACHE_SIZE            0

    /* ������󻺴�ѭ�����ݵ�������
     * ����1/4Բ���ܳ����ڿ����
     * �뾶*ÿ��Բʹ��4���ֽ�(������õİ뾶)
     * 0:���û��� */
    #define LV_CIRCLE_CACHE_SIZE            4
    
#endif /*LV_DRAW_COMPLEX*/

/* Ĭ��ͼ�񻺴��С��ͼ�񻺴汣��ͼ��򿪡�
 * ���ֻʹ�����õ�ͼ���ʽ������û�����������ơ�(��û�������µ�ͼ�������)
 * ���ӵ�ͼ�������(��PNG��JPG)������Ա���������/�����ͼ��Ȼ�����򿪵�ͼ����ܻ����Ķ����RAM��
 * 0:���û��� */
#define LV_IMG_CACHE_DEF_SIZE               0

/* ÿ���¶�����ͣ������Ŀ���������ֵ����������ͣվ��
 * ÿ�������ֹͣ����(sizeof(lv_color_t) + 1)�ֽ� */
#define LV_GRADIENT_MAX_STOPS               2

/* Ĭ���ݶȻ�������С��
 * ��LVGL�����ݶȡ���ͼ���������Խ����Ǳ��浽���棬�Ա����ٴμ������ǡ�
 * LV_GRAD_CACHE_DEF_SIZE���û���Ĵ�С(���ֽ�Ϊ��λ)��
 * �������̫С����ͼֻ������Ҫ���Ƶ�ʱ�򱻷��䡣
 * 0��ʾû�л���*/
#define LV_GRAD_CACHE_DEF_SIZE              0

/* ������������(�����޵���ɫ�����ʾ��ʵ���Ӿ�ƽ������ɫ����)
 * LV_DITHER_GRADIENT��ζ�ŷ��������Ⱦ�����һ����������
 * �ڴ����ĵ�������(32λ*�������)����24λ*����������ʹ�ô�����ɢ */
#define LV_DITHER_GRADIENT                  0
#if LV_DITHER_GRADIENT
    /* �����˴�����ɢ������֧�֡�
     * ������ɢ�����õ��˸��õ��Ӿ�Ч�������ڻ���ʱ��ζ�Ÿ����CPU���ڴ����ġ��ڴ���������(24λ*����Ŀ���) */
    #define LV_DITHER_ERROR_DIFFUSION       0
#endif

/* Ϊ��ת�������󻺳�����С��������ʾ��������������������תʱʹ�� */
#define LV_DISP_ROT_MAX_BUF                 (10*1024)

/*-------------
 * 2. GPU
 *-----------*/

/* ʹ��STM32��DMA2D(����Chrom Art) GPU */
#define LV_USE_GPU_STM32_DMA2D              0
#if LV_USE_GPU_STM32_DMA2D
    /* ���붨�����Ŀ�괦������CMSISͷ��·��
       �硣��stm32f769xx.h����stm32f429xx.h��*/
    #define LV_GPU_DMA2D_CMSIS_INCLUDE
#endif

/* ʹ��NXP��PXP GPU iMX RTxxxƽ̨ */
#define LV_USE_GPU_NXP_PXP                  0
#if LV_USE_GPU_NXP_PXP
    /*1:ΪPXP (lv_gpu_nxp_pxp_osa.c)����Ĭ�ϵ�������FreeRTOS�жϴ�������
     *����lv_init()ʱ�Զ�����lv_gpu_nxp_pxp_init()��ע�����SDK_OS_FREE_RTOS
     *���붨���Ա�ʹ��FreeRTOS OSA������ѡ�������ʵ�֡�
     *0: lv_gpu_nxp_pxp_init()������lv_init()֮ǰ�ֶ�����
    */
    #define LV_USE_GPU_NXP_PXP_AUTO_INIT    0
#endif

/* ʹ��NXP��VG-Lite GPU iMX RTxxxƽ̨ */
#define LV_USE_GPU_NXP_VG_LITE              0

/* ʹ��SDL��Ⱦ��API */
#define LV_USE_GPU_SDL                      0
#if LV_USE_GPU_SDL
    #define LV_GPU_SDL_INCLUDE_PATH <SDL2/SDL.h>
    /* ���������С��Ĭ��8MB */
    #define LV_GPU_SDL_LRU_SIZE (1024 * 1024 * 8)
    /* �Զ�����ģʽ���ɰ���ƣ��������Ҫ���Ӿ�SDL2����� */
    #define LV_GPU_SDL_CUSTOM_BLEND_MODE (SDL_VERSION_ATLEAST(2, 0, 6))
#endif

/*-------------
 * 3. ��־
 *-----------*/

/* ������־ģ�� */
#define LV_USE_LOG                          0
#if LV_USE_LOG

    /*Ӧ�����Ӷ���Ҫ����־:
    *LV_LOG_LEVEL_TRACE       ��������־��������ϸ����Ϣ
    *LV_LOG_LEVEL_INFO        ��¼��Ҫ�¼�
    *LV_LOG_LEVEL_WARN        ���������һЩ����Ҫ�����鵫û���������⣬���¼����
    *LV_LOG_LEVEL_ERROR       ֻ����ϵͳ���ܳ��ֹ���ʱ�Ż���ֹؼ�����
    *LV_LOG_LEVEL_USER        ���û��Լ����ӵ���־
    *LV_LOG_LEVEL_NONE        ��Ҫ��¼�κ�����*/
    #define LV_LOG_LEVEL LV_LOG_LEVEL_WARN

    /*1: ʹ��'printf'��ӡ��־;
     *0: �û���Ҫ��' lv_log_register_print_cb() 'ע��ص����� */
    #define LV_LOG_PRINTF                   0

    /* �ڲ���������־��ģ��������/����LV_LOG_TRACE */
    #define LV_LOG_TRACE_MEM                1
    #define LV_LOG_TRACE_TIMER              1
    #define LV_LOG_TRACE_INDEV              1
    #define LV_LOG_TRACE_DISP_REFR          1
    #define LV_LOG_TRACE_EVENT              1
    #define LV_LOG_TRACE_OBJ_CREATE         1
    #define LV_LOG_TRACE_LAYOUT             1
    #define LV_LOG_TRACE_ANIM               1

#endif  /*LV_USE_LOG*/

/*-------------
 * 4. ����
 *-----------*/

/* �������ʧ�ܻ�����Ч���ݣ������ö��ԡ�
 * ���������LV_USE_LOG��ʧ��ʱ���ӡ������Ϣ*/
#define LV_USE_ASSERT_NULL                  1   /* �������Ƿ�ΪNULL��(�ǳ���,�Ƽ�) */
#define LV_USE_ASSERT_MALLOC                1   /* ����ڴ��Ƿ����ɹ���(�ǳ���,�Ƽ�) */
#define LV_USE_ASSERT_STYLE                 0   /* �����ʽ�Ƿ���ȷ��ʼ����(�ǳ���,�Ƽ�) */
#define LV_USE_ASSERT_MEM_INTEGRITY         0   /* �ؼ�������ɺ����顰lv_mem���������ԡ�(��)*/
#define LV_USE_ASSERT_OBJ                   0   /* ����������ͺʹ���(���磬δɾ��)��(��) */

/* ��assert����ʱ������һ���Զ��崦������������������MCU */
#define LV_ASSERT_HANDLER_INCLUDE           <stdint.h>
#define LV_ASSERT_HANDLER while(1);         /* ֹͣ��Ĭ������� */

/*-------------
 * 5. ����
 *-----------*/

/* 1:��ʾCPUʹ���ʺ�FPS */
#define LV_USE_PERF_MONITOR                 0
#if LV_USE_PERF_MONITOR
    #define LV_USE_PERF_MONITOR_POS LV_ALIGN_BOTTOM_RIGHT
#endif

/* 1����ʾʹ�õ��ڴ���ڴ���Ƭ
 * Ҫ��LV_MEM_CUSTOM = 0*/
#define LV_USE_MEM_MONITOR                  0
#if LV_USE_MEM_MONITOR
    #define LV_USE_MEM_MONITOR_POS LV_ALIGN_BOTTOM_LEFT
#endif

/* 1:�����»��Ƶ������ϻ�������Ĳ�ɫ���� */
#define LV_USE_REFR_DEBUG                   0

/* �ı����õ�(v)snprintf���� */
#define LV_SPRINTF_CUSTOM                   0
#if LV_SPRINTF_CUSTOM
    #define LV_SPRINTF_INCLUDE  <stdio.h>
    #define lv_snprintf         snprintf
    #define lv_vsnprintf        vsnprintf
#else   /*LV_SPRINTF_CUSTOM*/
    #define LV_SPRINTF_USE_FLOAT            0
#endif  /*LV_SPRINTF_CUSTOM*/

#define LV_USE_USER_DATA                    1

/* �����ռ�������
 * ���lvgl�󶨵��߼����ԣ������ڴ��ɸ����Թ���ʱʹ��*/
#define LV_ENABLE_GC                        0
#if LV_ENABLE_GC != 0
    #define LV_GC_INCLUDE "gc.h"                           /* ���������ռ�����صĶ��� */
#endif /*LV_ENABLE_GC*/

 
 
/*********************************************************************************

                                        ����������
                                        
 ***********************************************************************************/
/* ��������Ϊ1�Ĵ����ϵͳ */
#define LV_BIG_ENDIAN_SYSTEM                0

/* Ϊ' lv_tick_inc '��������һ���Զ������� */
#define LV_ATTRIBUTE_TICK_INC

/* Ϊ' lv_timer_handler '��������һ���Զ������� */
#define LV_ATTRIBUTE_TIMER_HANDLER

/* Ϊ' lv_disp_flush_ready '��������һ���Զ������� */
#define LV_ATTRIBUTE_FLUSH_READY

/* ����������Ķ����С */
#define LV_ATTRIBUTE_MEM_ALIGN_SIZE         1

/* �������ӵ���Ҫ�����ڴ�ĵط�(Ĭ�������-Os���ݿ��ܲ�����뵽�߽�)��
 * ��__attribute__((����(4))) */
#define LV_ATTRIBUTE_MEM_ALIGN

/* ��������Ǵ��ͳ������飬���������λͼ */
#define LV_ATTRIBUTE_LARGE_CONST

/* RAM�д����������ı�����ǰ׺ */
#define LV_ATTRIBUTE_LARGE_RAM_ARRAY

/* �����ܹؼ����ܷ��������ڴ���(����RAM) */
#define LV_ATTRIBUTE_FAST_MEM

/* ��GPU���ٲ�����ʹ�õ�ǰ׺������ͨ����Ҫ������DMA�ɷ��ʵ�RAM���� */
#define LV_ATTRIBUTE_DMA

/* �������ͳ������󶨡��ú���LV_<CONST> that��ʽ�ĳ���һ��ʹ��
 * ҲӦ�ó�����LVGL��API����Micropython��*/
#define LV_EXPORT_CONST_INT(int_value) struct _silence_gcc_warning /* Ĭ��ֵֻ�Ƿ�ֹGCC���� */

/* ��չĬ��ֵ-32k..32k���귶Χ��-4M..ʹ��int32_t������int16_t��Ϊ���� */
#define LV_USE_LARGE_COORD                  0




/*********************************************************************************

                                        �ֿ�����
                                        
 ***********************************************************************************/
/* ���������������ASCII��Χ��һЩ����ʹ��bpp = 4
 * https://fonts.google.com/specimen/Montserrat */
#define LV_FONT_MONTSERRAT_8                0
#define LV_FONT_MONTSERRAT_10               0
#define LV_FONT_MONTSERRAT_12               1
#define LV_FONT_MONTSERRAT_14               1
#define LV_FONT_MONTSERRAT_16               1
#define LV_FONT_MONTSERRAT_18               0
#define LV_FONT_MONTSERRAT_20               1
#define LV_FONT_MONTSERRAT_22               0
#define LV_FONT_MONTSERRAT_24               0
#define LV_FONT_MONTSERRAT_26               0
#define LV_FONT_MONTSERRAT_28               0
#define LV_FONT_MONTSERRAT_30               0
#define LV_FONT_MONTSERRAT_32               0
#define LV_FONT_MONTSERRAT_34               0
#define LV_FONT_MONTSERRAT_36               0
#define LV_FONT_MONTSERRAT_38               0
#define LV_FONT_MONTSERRAT_40               0
#define LV_FONT_MONTSERRAT_42               0
#define LV_FONT_MONTSERRAT_44               0
#define LV_FONT_MONTSERRAT_46               0
#define LV_FONT_MONTSERRAT_48               0

/* չʾ��ɫ */
#define LV_FONT_MONTSERRAT_12_SUBPX         0
#define LV_FONT_MONTSERRAT_28_COMPRESSED    0  /* bpp = 3 */
#define LV_FONT_DEJAVU_16_PERSIAN_HEBREW    0  /* ϣ��������������˹���Լ����ǵĸ�����ʽ */
#define LV_FONT_SIMSUN_16_CJK               0  /* 1000�������CJK���ɻ�s */

/* ���������ĵ��ռ����� */
#define LV_FONT_UNSCII_8                    0
#define LV_FONT_UNSCII_16                   0

/* ��ѡ�����Զ������������
 * ��Ҳ����ʹ����Щ������ΪĬ�����壬���ǽ���ȫ����õġ�
 * �硣#define LV_FONT_CUSTOM_DECLARE LV_FONT_DECLARE(my_font_1) LV_FONT_DECLARE(my_font_2) */
#define LV_FONT_CUSTOM_DECLARE

/* ʼ������Ĭ������ */
#define LV_FONT_DEFAULT                     &lv_font_montserrat_14

/* ���ô����������/����д����ַ������塣
 * ����ȡ���������С���������bpp��
 * ���������󽫱����������������Ҫ�� */
#define LV_FONT_FMT_TXT_LARGE               0

/* ����/���ö�ѹ�������֧�֡� */
#define LV_USE_FONT_COMPRESSED              0

/* ʹ��������Ⱦ */
#define LV_USE_FONT_SUBPX                   0
#if LV_USE_FONT_SUBPX
    /* ������ʾ������˳��RGBͨ��������˳���롰�����������޹ء� */
    #define LV_FONT_SUBPX_BGR               0  /* 0: RGB;1: BGR���� */
#endif

/*********************************************************************************

                                        �ı�����
                                        
 ***********************************************************************************/
/**
 * Ϊ�ַ���ѡ���ַ�����
 * IDE��༭��Ӧ�þ�����ͬ���ַ�����
 * - LV_TXT_ENC_UTF8
 * - LV_TXT_ENC_ASCII
 */
#define LV_TXT_ENC LV_TXT_ENC_UTF8

/* ��������Щ�ַ����ж�(����)�ı� */
#define LV_TXT_BREAK_CHARS                  " ,.;:-_"

/* ���һ��������������ô�����ͻ��ڡ��������ĵط�����
 * Ҫ���ã�����һ��ֵ<= 0 */
#define LV_TXT_LINE_BREAK_LONG_LEN          0

/* ��һ���������У���ͣ��֮ǰ����һ�е���С�ַ�����
 * ȡ����LV_TXT_LINE_BREAK_LONG_LEN�� */
#define LV_TXT_LINE_BREAK_LONG_PRE_MIN_LEN  3

/* ��һ���������У���ͣ�ٺ����һ�е���С�ַ�����
 * ȡ����LV_TXT_LINE_BREAK_LONG_LEN��*/
#define LV_TXT_LINE_BREAK_LONG_POST_MIN_LEN 3

/* ���������ı�������ɫ�Ŀ����ַ��� */
#define LV_TXT_COLOR_CMD                    "#"

/* ֧��˫���ı���������ϴ����Һʹ��ҵ�����ı���
 * ��������Unicode˫���㷨���д���:
 * https://www.w3.org/International/articles/inline-bidi-markup/uba-basics*/
#define LV_USE_BIDI                         0
#if LV_USE_BIDI
    /* ����Ĭ�Ϸ���֧�ֵ�ֵ:
    *`LV_BASE_DIR_LTR` ������
    *`LV_BASE_DIR_RTL` ���ҵ���
    *`LV_BASE_DIR_AUTO` ����ı��������� */
    #define LV_BIDI_BASE_DIR_DEF LV_BASE_DIR_AUTO
#endif

/* ֧�ְ�������/��˹����
 * ����Щ�����У��ַ�Ӧ�ø��������ı��е�λ�ñ��滻Ϊ������ʽ */
#define LV_USE_ARABIC_PERSIAN_CHARS         0



/*********************************************************************************

                                        �ؼ�����
                                        
 ***********************************************************************************/
/* С�������ĵ�:https://docs.lvgl.io/latest/en/html/widgets/index.html */

#define LV_USE_ARC                          1

#define LV_USE_ANIMIMG                      1

#define LV_USE_BAR                          1

#define LV_USE_BTN                          1

#define LV_USE_BTNMATRIX                    1

#define LV_USE_CANVAS                       1

#define LV_USE_CHECKBOX                     1

#define LV_USE_DROPDOWN                     1   /* ����: lv_label */

#define LV_USE_IMG                          1   /* ����: lv_label */

#define LV_USE_LABEL                        1
#if LV_USE_LABEL
    #define LV_LABEL_TEXT_SELECTION         1  /* ���ñ�ǩ��ѡ���ı�*/
    #define LV_LABEL_LONG_TXT_HINT          1  /* �ڱ�ǩ�д洢һЩ�������Ϣ���Լӿ���Ʒǳ������ı� */
#endif

#define LV_USE_LINE                         1

#define LV_USE_ROLLER                       1  /* ����: lv_label */
#if LV_USE_ROLLER
    #define LV_ROLLER_INF_PAGES             7  /* ����Ͳ����ʱ������ġ�ҳ���� */
#endif

#define LV_USE_SLIDER                       1  /* ����: lv_bar*/

#define LV_USE_SWITCH                       1

#define LV_USE_TEXTAREA                     1  /* ����: lv_label*/
#if LV_USE_TEXTAREA != 0
    #define LV_TEXTAREA_DEF_PWD_SHOW_TIME   1500    /*ms*/
#endif

#define LV_USE_TABLE                        1


/*********************************************************************************

                                        �ر���
                                        
 ***********************************************************************************/
/*-----------
 * 1. �ؼ�
 *----------*/
#define LV_USE_CALENDAR                     1
#if LV_USE_CALENDAR
    #define LV_CALENDAR_WEEK_STARTS_MONDAY  0
    #if LV_CALENDAR_WEEK_STARTS_MONDAY
        #define LV_CALENDAR_DEFAULT_DAY_NAMES {"Mo", "Tu", "We", "Th", "Fr", "Sa", "Su"}
    #else
        #define LV_CALENDAR_DEFAULT_DAY_NAMES {"Su", "Mo", "Tu", "We", "Th", "Fr", "Sa"}
    #endif

    #define LV_CALENDAR_DEFAULT_MONTH_NAMES {"January", "February", "March",  "April", "May",  "June", "July", "August", "September", "October", "November", "December"}
    #define LV_USE_CALENDAR_HEADER_ARROW    1
    #define LV_USE_CALENDAR_HEADER_DROPDOWN 1
#endif  /*LV_USE_CALENDAR*/

#define LV_USE_CHART                        1

#define LV_USE_COLORWHEEL                   1

#define LV_USE_IMGBTN                       1

#define LV_USE_KEYBOARD                     1

#define LV_USE_LED                          1

#define LV_USE_LIST                         1

#define LV_USE_MENU                         1

#define LV_USE_METER                        1

#define LV_USE_MSGBOX                       1

#define LV_USE_SPINBOX                      1

#define LV_USE_SPINNER                      1

#define LV_USE_TABVIEW                      1

#define LV_USE_TILEVIEW                     1

#define LV_USE_WIN                          1

#define LV_USE_SPAN                         1
#if LV_USE_SPAN
    /* һ�����ı����԰������������span������ */
    #define LV_SPAN_SNIPPET_STACK_SIZE      64
#endif

/*-----------
 * 2. ����
 *----------*/

/* һ���򵥣�����ӡ����̺ͷǳ����������� e*/
#define LV_USE_THEME_DEFAULT                1
#if LV_USE_THEME_DEFAULT

    /* 0:��ģʽ;1:�ڰ���ģʽ */
    #define LV_THEME_DEFAULT_DARK           0

    /* 1:����ʹ������ */
    #define LV_THEME_DEFAULT_GROW           1

    /* Ĭ��ת��ʱ��[ms] */
    #define LV_THEME_DEFAULT_TRANSITION_TIME 80
#endif /*LV_USE_THEME_DEFAULT*/

/* һ���ǳ��򵥵����⣬����һ���Զ��������������� */
#define LV_USE_THEME_BASIC                  1

/* רΪ��ɫ��ʾ����Ƶ����� */
#define LV_USE_THEME_MONO                   1

/*-----------
 * 3. ����
 *----------*/

/* ������CSS�е�Flexbox���� */
#define LV_USE_FLEX                         1

/* ������CSS�е����񲼾� */
#define LV_USE_GRID                         1

/*---------------------
 * 4. ��������
 *--------------------*/

/* ����ͨ��api���ļ�ϵͳ�ӿ�
 * Ϊ��API����������������� */
 
/* STDIO */
#define LV_USE_FS_STDIO            0
#if LV_USE_FS_STDIO
    #define LV_FS_STDIO_LETTER      '\0'        /* ����һ���ɷ����������Ĵ�д��ĸ(���硣��A��) */
    #define LV_FS_STDIO_PATH        ""          /* ���ù���Ŀ¼���ļ�/Ŀ¼·������׷�ӵ��� */
    #define LV_FS_STDIO_CACHE_SIZE  0           /* >0��lv_fs_read()�л�������ֽ��� */
#endif

/* POSIX */
#define LV_USE_FS_POSIX             0
#if LV_USE_FS_POSIX
    #define LV_FS_POSIX_LETTER      '\0'        /* ����һ���ɷ����������Ĵ�д��ĸ(���硣��A��) */
    #define LV_FS_POSIX_PATH        ""          /* ���ù���Ŀ¼���ļ�/Ŀ¼·������׷�ӵ��� */
    #define LV_FS_POSIX_CACHE_SIZE  0           /* >0��lv_fs_read()�л�������ֽ��� */
#endif

/* WIN32 */
#define LV_USE_FS_WIN32             0
#if LV_USE_FS_WIN32
    #define LV_FS_WIN32_LETTER      '\0'        /* ����һ���ɷ����������Ĵ�д��ĸ(���硣��A��) */
    #define LV_FS_WIN32_PATH        ""          /* ���ù���Ŀ¼���ļ�/Ŀ¼·������׷�ӵ��� */
    #define LV_FS_WIN32_CACHE_SIZE  0           /* >0��lv_fs_read()�л�������ֽ��� */
#endif

/* FATFS */
#define LV_USE_FS_FATFS             0
#if LV_USE_FS_FATFS
    #define LV_FS_FATFS_LETTER      '\0'        /* ����һ���ɷ����������Ĵ�д��ĸ(���硣��A��) */
    #define LV_FS_FATFS_CACHE_SIZE  0           /* >0��lv_fs_read()�л�������ֽ��� */
#endif

/* PNG�������� */
#define LV_USE_PNG                          0

/* BMP �������� */
#define LV_USE_BMP                          0

/* JPG +�ָ�JPG�������⡣
 * Split JPG��ΪǶ��ʽϵͳ�Ż����Զ����ʽ */
#define LV_USE_SJPG                         0

/* GIF�������� */
#define LV_USE_GIF                          1

/* QR�������� */
#define LV_USE_QRCODE                       0

/* FreeType�� */
#define LV_USE_FREETYPE                     0
#if LV_USE_FREETYPE
    /* FreeType���ڻ����ַ�[bytes]���ڴ�(-1:û�л���) */
    #define LV_FREETYPE_CACHE_SIZE          (16 * 1024)
    #if LV_FREETYPE_CACHE_SIZE >= 0
        /* 1:λͼcacheʹ��sbit cache, 0:λͼcacheʹ��ͼ��cache
         * sbit����:����С��λͼ(�����С< 256)�������ڴ�Ч�ʸ���
         *��������С>= 256����������Ϊͼ�񻺴� */
        #define LV_FREETYPE_SBIT_CACHE      0
        /* ���������ʵ�������Ĵ򿪵�FT_Face/FT_Size��������������*��
           (0:ʹ��ϵͳĬ��ֵ) */
        #define LV_FREETYPE_CACHE_FT_FACES  0
        #define LV_FREETYPE_CACHE_FT_SIZES  0
    #endif
#endif

/* Rlottie �� */
#define LV_USE_RLOTTIE                      0

/* FFmpeg���ͼ�����Ͳ�����Ƶ
 * ֧��������Ҫ��ͼ���ʽ�����Բ���������ͼ�����������t*/
#define LV_USE_FFMPEG                       0
#if LV_USE_FFMPEG
    /* ��������Ϣת����stderr */
    #define LV_FFMPEG_AV_DUMP_FORMAT        0
#endif

/*-----------
 * 5. ����
 *----------*/

/* 1:����API�Զ�����п��� */
#define LV_USE_SNAPSHOT                     1

/* 1:ʹ��Monkey���� */
#define LV_USE_MONKEY                       0

/* 1:�������񵼺� */
#define LV_USE_GRIDNAV                      0

/*********************************************************************************

                                        ʵ��
                                        
 ***********************************************************************************/
/* �����ÿ⹹��ʾ�� */
#define LV_BUILD_EXAMPLES                   1

/*===================
 * ��ʾʹ��
 ====================*/

/* ��ʾ��һЩ������������Ҫ���ӡ�LV_MEM_SIZE�� */
#define LV_USE_DEMO_WIDGETS                 0
#if LV_USE_DEMO_WIDGETS
#define LV_DEMO_WIDGETS_SLIDESHOW           0
#endif

/* ��ʾ�������ͼ��̵��÷� */
#define LV_USE_DEMO_KEYPAD_AND_ENCODER      0

/* ��׼ϵͳ */
#define LV_USE_DEMO_BENCHMARK               0

/* LVGLѹ������ */
#define LV_USE_DEMO_STRESS                  0

/* ���ֲ���������ʾ */
#define LV_USE_DEMO_MUSIC                   1
#if LV_USE_DEMO_MUSIC
# define LV_DEMO_MUSIC_SQUARE               0
# define LV_DEMO_MUSIC_LANDSCAPE            0
# define LV_DEMO_MUSIC_ROUND                0
# define LV_DEMO_MUSIC_LARGE                0
# define LV_DEMO_MUSIC_AUTO_PLAY            0
#endif

/*--END OF LV_CONF_H--*/

#endif /*LV_CONF_H*/

#endif /*End of "Content enable"*/
