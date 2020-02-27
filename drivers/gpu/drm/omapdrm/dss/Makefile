# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_OMAP2_DSS) += omapdss.o
# Core DSS files
omapdss-y := base.o dss.o dispc.o display.o dispc_coefs.o output.o \
	pll.o video-pll.o
omapdss-$(CONFIG_OMAP2_DSS_DPI) += dpi.o
omapdss-$(CONFIG_OMAP2_DSS_VENC) += venc.o
omapdss-$(CONFIG_OMAP2_DSS_SDI) += sdi.o
omapdss-$(CONFIG_OMAP2_DSS_DSI) += dsi.o
omapdss-$(CONFIG_OMAP2_DSS_HDMI_COMMON) += hdmi_common.o hdmi_wp.o hdmi_pll.o \
	hdmi_phy.o
omapdss-$(CONFIG_OMAP4_DSS_HDMI) += hdmi4.o hdmi4_core.o
omapdss-$(CONFIG_OMAP4_DSS_HDMI_CEC) += hdmi4_cec.o
omapdss-$(CONFIG_OMAP5_DSS_HDMI) += hdmi5.o hdmi5_core.o
ccflags-$(CONFIG_OMAP2_DSS_DEBUG) += -DDEBUG
