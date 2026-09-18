/*
 * SGI video-source control interface.
 *
 * A device that can route a host media source (a local video file or a
 * stream URL) to one of its video inputs implements this interface.  The
 * UI (ui/gtk.c) looks the interface up generically so that it can build
 * its "Video" menu without knowing which machine or device is in use;
 * machines whose devices do not implement it simply get no menu.
 *
 * The interface is deliberately UI-agnostic: an implementation starts and
 * stops whatever external decoder it needs, and the UI only asks it to
 * attach a named input, detach it, and report the current state.
 *
 * Copyright (c) 2025 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_VIDEO_SOURCE_H
#define HW_MISC_SGI_VIDEO_SOURCE_H

#include "qom/object.h"

#define TYPE_SGI_VIDEO_SOURCE "sgi-video-source"

typedef struct SGIVideoSourceClass SGIVideoSourceClass;
DECLARE_CLASS_CHECKERS(SGIVideoSourceClass, SGI_VIDEO_SOURCE,
                       TYPE_SGI_VIDEO_SOURCE)
#define SGI_VIDEO_SOURCE(obj) \
    INTERFACE_CHECK(SGIVideoSource, (obj), TYPE_SGI_VIDEO_SOURCE)

typedef struct SGIVideoSource SGIVideoSource;

/**
 * SGIVideoSourceClass:
 *
 * @attach: route @source (a local file path, or a URL when @is_url) to the
 *          video input named @input (e.g. "vin1"); return false and set
 *          @errp on failure.
 * @detach: stop the source currently attached to @input (idempotent).
 * @is_attached: whether a source is currently attached to @input.
 * @describe: a human-readable description of the attached source, or NULL.
 */
struct SGIVideoSourceClass {
    InterfaceClass parent;

    bool (*attach)(SGIVideoSource *src, const char *input,
                   const char *source, bool is_url, Error **errp);
    void (*detach)(SGIVideoSource *src, const char *input);
    bool (*is_attached)(SGIVideoSource *src, const char *input);
    const char *(*describe)(SGIVideoSource *src, const char *input);
};

/*
 * The UI locates an implementation generically with
 * object_resolve_path_type("", TYPE_SGI_VIDEO_SOURCE, NULL); #TYPE_SGI_VIDEO_SOURCE
 * itself is registered by the device that implements it.
 */

#endif /* HW_MISC_SGI_VIDEO_SOURCE_H */
