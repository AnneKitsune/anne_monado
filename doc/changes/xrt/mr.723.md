---
- mr.754
- mr.807
---
Make `xrt_swapchain` be reference counted. This will greatly help with
handling swapchains for multiple clients in the compositor rendering pipeline
where a client might go away while the compositor is using it.
