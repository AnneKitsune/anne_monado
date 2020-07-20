// Copyright 2019-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Swapchain code for the main compositor.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup comp_main
 */

#include "util/u_misc.h"

#include "main/comp_compositor.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#if defined(XRT_GRAPHICS_BUFFER_HANDLE_IS_AHARDWAREBUFFER)
#include <android/hardware_buffer.h>
#endif


/*
 *
 * Swapchain member functions.
 *
 */

static void
swapchain_destroy(struct xrt_swapchain *xsc)
{
	struct comp_swapchain *sc = comp_swapchain(xsc);

	COMP_SPEW(sc->c, "DESTROY");

	u_threading_stack_push(&sc->c->threading.destroy_swapchains, sc);
}

static xrt_result_t
swapchain_acquire_image(struct xrt_swapchain *xsc, uint32_t *out_index)
{
	struct comp_swapchain *sc = comp_swapchain(xsc);

	COMP_SPEW(sc->c, "ACQUIRE_IMAGE");

	// Returns negative on empty fifo.
	int res = u_index_fifo_pop(&sc->fifo, out_index);
	if (res >= 0) {
		return XRT_SUCCESS;
	}
	return XRT_ERROR_NO_IMAGE_AVAILABLE;
}

static xrt_result_t
swapchain_wait_image(struct xrt_swapchain *xsc,
                     uint64_t timeout,
                     uint32_t index)
{
	struct comp_swapchain *sc = comp_swapchain(xsc);

	COMP_SPEW(sc->c, "WAIT_IMAGE");
	return XRT_SUCCESS;
}

static xrt_result_t
swapchain_release_image(struct xrt_swapchain *xsc, uint32_t index)
{
	struct comp_swapchain *sc = comp_swapchain(xsc);

	COMP_SPEW(sc->c, "RELEASE_IMAGE");

	int res = u_index_fifo_push(&sc->fifo, index);

	if (res >= 0) {
		return XRT_SUCCESS;
	}
	// FIFO full
	return XRT_ERROR_NO_IMAGE_AVAILABLE;
}


/*
 *
 * Helper functions.
 *
 */

static struct comp_swapchain *
alloc_and_set_funcs(struct comp_compositor *c, uint32_t num_images)
{
	struct comp_swapchain *sc = U_TYPED_CALLOC(struct comp_swapchain);
	sc->base.base.destroy = swapchain_destroy;
	sc->base.base.acquire_image = swapchain_acquire_image;
	sc->base.base.wait_image = swapchain_wait_image;
	sc->base.base.release_image = swapchain_release_image;
	sc->base.base.num_images = num_images;
	sc->c = c;

	// Make sure the handles are invalid.
	for (uint32_t i = 0; i < ARRAY_SIZE(sc->base.images); i++) {
		sc->base.images[i].handle = XRT_GRAPHICS_BUFFER_HANDLE_INVALID;
	}

	return sc;
}

static void
do_post_create_vulkan_setup(struct comp_compositor *c,
                            struct comp_swapchain *sc)
{
	struct xrt_swapchain_create_info *info = &sc->vkic.info;
	uint32_t num_images = sc->vkic.num_images;
	VkCommandBuffer cmd_buffer;

	VkComponentMapping components = {
	    .r = VK_COMPONENT_SWIZZLE_R,
	    .g = VK_COMPONENT_SWIZZLE_G,
	    .b = VK_COMPONENT_SWIZZLE_B,
	    .a = VK_COMPONENT_SWIZZLE_ONE,
	};

	for (uint32_t i = 0; i < num_images; i++) {
		sc->images[i].views.alpha =
		    U_TYPED_ARRAY_CALLOC(VkImageView, info->array_size);
		sc->images[i].views.no_alpha =
		    U_TYPED_ARRAY_CALLOC(VkImageView, info->array_size);
		sc->images[i].array_size = info->array_size;

		vk_create_sampler(&c->vk, &sc->images[i].sampler);

		for (uint32_t layer = 0; layer < info->array_size; ++layer) {
			VkImageSubresourceRange subresource_range = {
			    .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
			    .baseMipLevel = 0,
			    .levelCount = 1,
			    .baseArrayLayer = layer,
			    .layerCount = 1,
			};

			vk_create_view(&c->vk, sc->vkic.images[i].handle,
			               (VkFormat)info->format,
			               subresource_range,
			               &sc->images[i].views.alpha[layer]);
			vk_create_view_swizzle(
			    &c->vk, sc->vkic.images[i].handle,
			    (VkFormat)info->format, subresource_range,
			    components, &sc->images[i].views.no_alpha[layer]);
		}
	}

	// Prime the fifo
	for (uint32_t i = 0; i < num_images; i++) {
		u_index_fifo_push(&sc->fifo, i);
	}


	/*
	 *
	 * Transition image.
	 *
	 */

	vk_init_cmd_buffer(&c->vk, &cmd_buffer);

	VkImageSubresourceRange subresource_range = {
	    .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	    .baseMipLevel = 0,
	    .levelCount = 1,
	    .baseArrayLayer = 0,
	    .layerCount = info->array_size,
	};

	for (uint32_t i = 0; i < num_images; i++) {
		vk_set_image_layout(
		    &c->vk, cmd_buffer, sc->vkic.images[i].handle, 0,
		    VK_ACCESS_SHADER_READ_BIT, VK_IMAGE_LAYOUT_UNDEFINED,
		    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
		    subresource_range);
	}

	vk_submit_cmd_buffer(&c->vk, cmd_buffer);
}

static void
clean_image_views(struct vk_bundle *vk,
                  size_t array_size,
                  VkImageView **views_ptr)
{
	VkImageView *views = *views_ptr;
	if (views == NULL) {
		return;
	}

	for (uint32_t i = 0; i < array_size; ++i) {
		if (views[i] == VK_NULL_HANDLE) {
			continue;
		}

		vk->vkDestroyImageView(vk->device, views[i], NULL);
		views[i] = VK_NULL_HANDLE;
	}

	free(views);
	array_size = 0;

	*views_ptr = NULL;
}

/*!
 * Free and destroy any initialized fields on the given image, safe to pass in
 * images that has one or all fields set to NULL.
 */
static void
image_cleanup(struct vk_bundle *vk, struct comp_swapchain_image *image)
{
	/*
	 * This makes sure that any pending command buffer has completed and all
	 * resources referred by it can now be manipulated. This make sure that
	 * validation doesn't complain. This is done during image destruction so
	 * isn't time critical.
	 */
	vk->vkDeviceWaitIdle(vk->device);

	clean_image_views(vk, image->array_size, &image->views.alpha);
	clean_image_views(vk, image->array_size, &image->views.no_alpha);

	if (image->sampler != VK_NULL_HANDLE) {
		vk->vkDestroySampler(vk->device, image->sampler, NULL);
		image->sampler = VK_NULL_HANDLE;
	}
}

/*
 *
 * Exported functions.
 *
 */

xrt_result_t
comp_swapchain_create(struct xrt_compositor *xc,
                      const struct xrt_swapchain_create_info *info,
                      struct xrt_swapchain **out_xsc)
{
	struct comp_compositor *c = comp_compositor(xc);
	uint32_t num_images = 3;
	VkResult ret;

	if ((info->create & XRT_SWAPCHAIN_CREATE_STATIC_IMAGE) != 0) {
		num_images = 1;
	}

	struct comp_swapchain *sc = alloc_and_set_funcs(c, num_images);

	COMP_DEBUG(c, "CREATE %p %dx%d", (void *)sc, info->width, info->height);

	// Use the image helper to allocate the images.
	ret = vk_ic_allocate(&c->vk, info, num_images, &sc->vkic);
	if (ret != VK_SUCCESS) {
		free(sc);
		return XRT_ERROR_VULKAN;
	}

	xrt_graphics_buffer_handle_t handles[ARRAY_SIZE(sc->vkic.images)];

	vk_ic_get_handles(&c->vk, &sc->vkic, ARRAY_SIZE(handles), handles);
	for (uint32_t i = 0; i < sc->vkic.num_images; i++) {
		sc->base.images[i].handle = handles[i];
		sc->base.images[i].size = sc->vkic.images[i].size;
	}

	do_post_create_vulkan_setup(c, sc);

	*out_xsc = &sc->base.base;

	return XRT_SUCCESS;
}

xrt_result_t
comp_swapchain_import(struct xrt_compositor *xc,
                      const struct xrt_swapchain_create_info *info,
                      struct xrt_image_native *native_images,
                      uint32_t num_images,
                      struct xrt_swapchain **out_xsc)
{
	struct comp_compositor *c = comp_compositor(xc);
	VkResult ret;

	struct comp_swapchain *sc = alloc_and_set_funcs(c, num_images);

	COMP_DEBUG(c, "CREATE FROM NATIVE %p %dx%d", (void *)sc, info->width,
	           info->height);

	// Use the image helper to get the images.
	ret = vk_ic_from_natives(&c->vk, info, native_images, num_images,
	                         &sc->vkic);
	if (ret != VK_SUCCESS) {
		return XRT_ERROR_VULKAN;
	}

	do_post_create_vulkan_setup(c, sc);

	*out_xsc = &sc->base.base;

	return XRT_SUCCESS;
}

void
comp_swapchain_really_destroy(struct comp_swapchain *sc)
{
	struct vk_bundle *vk = &sc->c->vk;

	COMP_SPEW(sc->c, "REALLY DESTROY");

	for (uint32_t i = 0; i < sc->base.base.num_images; i++) {
		image_cleanup(vk, &sc->images[i]);
	}

	for (uint32_t i = 0; i < sc->base.base.num_images; i++) {
		if (!xrt_graphics_buffer_is_valid(sc->base.images[i].handle)) {
			continue;
		}
//! @todo move to helper functions - move them out of IPC
#if defined(XRT_GRAPHICS_BUFFER_HANDLE_IS_AHARDWAREBUFFER)
		AHardwareBuffer_release(sc->base.images[i].handle);
#elif defined(XRT_GRAPHICS_BUFFER_HANDLE_IS_FD)
		close(sc->base.images[i].handle);
#else
#error "need port"
#endif
		sc->base.images[i].handle = XRT_GRAPHICS_BUFFER_HANDLE_INVALID;
	}

	vk_ic_destroy(vk, &sc->vkic);

	free(sc);
}
