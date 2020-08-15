/*
  This sample illustrates how to access the different camera
  parameter types. It uses the low-level functions provided by GenApiC
  instead of those provided by pylonC.
*/

#include <stdlib.h>
#include <stdio.h>
#include <malloc.h>

#include <pylonc/PylonC.h>

#define STRING_BUFFER_SIZE  512

#define CHECK(errc) if (GENAPI_E_OK != errc) printErrorAndExit(errc)

/* This function can be used to wait for user input at the end of the sample program. */
void pressEnterToExit(void);

/* This function demonstrates how to retrieve the error message for the last failed
   function call. */
static void
printErrorAndExit(GENAPIC_RESULT errc)
{
    char *errMsg;
    size_t length;

    /* Retrieve the error message.
    ... First find out how big the buffer must be, */
    GenApiGetLastErrorMessage(NULL, &length);
    errMsg = (char*) malloc(length);
    /* ... and retrieve the message. */
    GenApiGetLastErrorMessage(errMsg, &length);

    fprintf(stderr, "%s (%#08x).\n", errMsg, (unsigned int) errc);
    free(errMsg);

    /* Retrieve more details about the error
    ... First find out how big the buffer must be, */
    GenApiGetLastErrorDetail(NULL, &length);
    errMsg = (char*) malloc(length);
    /* ... and retrieve the message. */
    GenApiGetLastErrorDetail(errMsg, &length);

    fprintf(stderr, "%s\n", errMsg);
    free(errMsg);

    PylonTerminate();  /* Releases all pylon resources. */
    pressEnterToExit();

    exit(EXIT_FAILURE);
}

/* This function demonstrates how to check the presence, readability, and writability
   of a feature. */
static void
demonstrateAccessibilityCheck(PYLON_DEVICE_HANDLE hDev)
{
    NODEMAP_HANDLE      hNodeMap;
    NODE_HANDLE         hNode;
    const char          * pFeatureName;
    _Bool                val, val_read, val_write;
    GENAPIC_RESULT      res;

    /* Get a handle for the device's node map. */
    res = PylonDeviceGetNodeMap(hDev, &hNodeMap);
    CHECK(res);

    /* Check to see if a feature is implemented at all. The 'Width' feature is likely to
    be implemented by just about every existing camera. */
    pFeatureName = "Width";
    res = GenApiNodeMapGetNode(hNodeMap, pFeatureName, &hNode);
    CHECK(res);
    if(GENAPIC_INVALID_HANDLE != hNode)
    {
        /* Node exists, check whether feature is implemented. */
        res = GenApiNodeIsImplemented(hNode, &val);
        CHECK(res);
    }
    else
    {
        /* Node does not exist --> feature is not implemented. */
        val = 0;
    }
    printf("The '%s' feature %s implemented\n", pFeatureName, val ? "is" : "is not");

    /* This feature most likely does not exist */
    pFeatureName = "Weirdness";
    res = GenApiNodeMapGetNode(hNodeMap, pFeatureName, &hNode);
    CHECK(res);
    if(GENAPIC_INVALID_HANDLE != hNode)
    {
        /* Node exists, check whether feature is implemented. */
        res = GenApiNodeIsImplemented(hNode, &val);
        CHECK(res);
    }
    else
    {
        /* Node does not exist --> feature is not implemented. */
        val = 0;
    }
    printf("The '%s' feature %s implemented\n", pFeatureName, val ? "is" : "is not");


    /* Although a feature is implemented by the device, it may not be available
       with the device in its current state. Check to see if the feature is currently
       available. The GenApiNodeIsAvailable sets val to 0 if either the feature
       is not implemented or if the feature is not currently available. */
    pFeatureName = "BinningVertical";
    res = GenApiNodeMapGetNode(hNodeMap, pFeatureName, &hNode);
    CHECK(res);
    if(GENAPIC_INVALID_HANDLE != hNode)
    {
        /* Node exists, check whether feature is available. */
        res = GenApiNodeIsAvailable(hNode, &val);
        CHECK(res);
    }
    else
    {
        /* Node does not exist --> feature is not implemented, and hence not available. */
        val = 0;
    }
    printf("The '%s' feature %s available\n", pFeatureName, val ? "is" : "is not");

    /* If a feature is available, it could be read-only, write-only, or both
       readable and writable. Use the GenApiNodeIsReadable() and the
       GenApiNodeIsReadable() functions(). It is safe to call these functions
       for features that are currently not available or not implemented by the device.
       A feature that is not available or not implemented is neither readable nor writable.
       The readability and writability of a feature can change depending on the current
       state of the device. For example, the Width parameter might not be writable when
       the camera is acquiring images. */

    pFeatureName = "Width";
    res = GenApiNodeMapGetNode(hNodeMap, pFeatureName, &hNode);
    CHECK(res);
    if(GENAPIC_INVALID_HANDLE != hNode)
    {
        /* Node exists, check whether feature is readable. */
        res = GenApiNodeIsReadable(hNode, &val_read);
        CHECK(res);
        res = GenApiNodeIsReadable(hNode, &val_write);
        CHECK(res);
    }
    else
    {
        /* Node does not exist --> feature is neither readable nor witable. */
        val_read = val_write = 0;
    }
    printf("The '%s' feature %s readable\n", pFeatureName, val_read ? "is" : "is not");
    printf("The '%s' feature %s writable\n", pFeatureName, val_write ? "is" : "is not");
    printf("\n");
}


/* This function demonstrates how to handle integer camera parameters. */
static void
demonstrateIntFeature(PYLON_DEVICE_HANDLE hDev)
{
    NODEMAP_HANDLE      hNodeMap;
    NODE_HANDLE         hNode;
    static const char   featureName[] = "Width";  /* Name of the feature used in this sample: AOI Width. */
    int64_t             val, min, max, incr;      /* Properties of the feature. */
    GENAPIC_RESULT      res;                      /* Return value. */
    EGenApiNodeType     nodeType;
    _Bool                bval;

    /* Get a handle for the device's node map. */
    res = PylonDeviceGetNodeMap(hDev, &hNodeMap);
    CHECK(res);

    /* Look up the feature node */
    res = GenApiNodeMapGetNode(hNodeMap, featureName, &hNode);
    CHECK(res);
    if (GENAPIC_INVALID_HANDLE == hNode)
    {
        fprintf(stderr, "There is no feature named '%s'\n", featureName);
        return;
    }

    /* We want an integer feature node. */
    res = GenApiNodeGetType(hNode, &nodeType);
    CHECK(res);
    if (IntegerNode != nodeType)
    {
        fprintf(stderr, "'%s' is not an integer feature\n", featureName);
        return;
    }

    /*
       Query the current value, the range of allowed values, and the increment of the feature.
       For some integer features, you are not allowed to set every value within the
       value range. For example, for some cameras the Width parameter must be a multiple
       of 2. These constraints are expressed by the increment value. Valid values
       follow the rule: val >= min && val <= max && val == min + n * inc.
    */

    res = GenApiNodeIsReadable(hNode, &bval);
    CHECK(res);

    if (bval)
    {
        res = GenApiIntegerGetMin(hNode, &min);       /* Get the minimum value. */
        CHECK(res);
        res = GenApiIntegerGetMax(hNode, &max);       /* Get the maximum value. */
        CHECK(res);
        res = GenApiIntegerGetInc(hNode, &incr);       /* Get the increment value. */
        CHECK(res);
        res = GenApiIntegerGetValue(hNode, &val);     /* Get the current value. */
        CHECK(res);

#if __STDC_VERSION__ >= 199901L || defined(__GNUC__)
        printf("%s: min= %lld  max= %lld  incr=%lld  Value=%lld\n", featureName, (long long) min, (long long) max, (long long) incr, (long long) val);
#else
        printf("%s: min= %I64d  max= %I64d  incr=%I64d  Value=%I64d\n", featureName, min, max, incr, val);
#endif

        res = GenApiNodeIsWritable(hNode, &bval);
        CHECK(res);

        if (bval)
        {
            /* Set the Width half-way between minimum and maximum. */
            res = GenApiIntegerSetValue(hNode, min + (max - min) / incr / 2 * incr);
            CHECK(res);
        }
        else
            fprintf(stderr, "Cannot set value for feature '%s' - node not writable\n", featureName);
    }
    else
        fprintf(stderr, "Cannot read feature '%s' - node not readable\n", featureName);
}


/* Some features involve floating point parameters. This function illustrates how to set and get floating
   point parameters. */
static void
demonstrateFloatFeature(PYLON_DEVICE_HANDLE hDev)
{
    NODEMAP_HANDLE      hNodeMap;
    NODE_HANDLE         hNode;
    static const char   featureName[] = "Gamma";  /* The name of the feature used. */
    _Bool                bval;                     /* Is the feature available? */
    double              min, max, value;          /* Value range and current value. */
    EGenApiNodeType     nodeType;
    GENAPIC_RESULT      res;                      /* Return value. */

    /* Get a handle for the device's node map */
    res = PylonDeviceGetNodeMap(hDev, &hNodeMap);
    CHECK(res);

    /* Look up the feature node. */
    res = GenApiNodeMapGetNode(hNodeMap, featureName, &hNode);
    CHECK(res);
    if (GENAPIC_INVALID_HANDLE == hNode)
    {
        fprintf(stderr, "There is no feature named '%s'\n", featureName);
        return;
    }

    /* We want a float feature node. */
    res = GenApiNodeGetType(hNode, &nodeType);
    CHECK(res);
    if (FloatNode != nodeType)
    {
        fprintf(stderr, "'%s' is not an floating-point feature\n", featureName);
        return;
    }

    res = GenApiNodeIsReadable(hNode, &bval);
    CHECK(res);

    if (bval)
    {
        /* Query the value range and the current value. */
        res = GenApiFloatGetMin(hNode, &min);
        CHECK(res);
        res = GenApiFloatGetMax(hNode, &max);
        CHECK(res);
        res = GenApiFloatGetValue(hNode, &value);
        CHECK(res);

        printf("%s: min = %4.2f, max = %4.2f, value = %4.2f\n", featureName, min, max, value);

        /* Set a new value. */
        GenApiNodeIsWritable(hNode, &bval);
        CHECK(res);

        if (bval)
        {
            value = 0.5 * (min + max);
            printf("Setting %s to %4.2f\n", featureName, value);
            res = GenApiFloatSetValue(hNode, value);
            CHECK(res);
        }
        else
            printf("Cannot set value for feature '%s' - node not writable\n", featureName);
    }
    else
        printf("Cannot read feature '%s' - node not readable\n", featureName);
}


/* Some features are boolean features that can be switched on and off.
   This function illustrates how to access boolean features. */
static void
demonstrateBooleanFeature(PYLON_DEVICE_HANDLE hDev)
{
    NODEMAP_HANDLE      hNodeMap;
    NODE_HANDLE         hNode;
    static const char   featureName[] = "GammaEnable"; /* The name of the feature. */
    _Bool                value,bval;                    /* The value of the feature. */
    EGenApiNodeType     nodeType;
    GENAPIC_RESULT      res;                           /* Return value. */

    /* Get a handle for the device's node map. */
    res = PylonDeviceGetNodeMap(hDev, &hNodeMap);
    CHECK(res);

    /* Look up the feature node. */
    res = GenApiNodeMapGetNode(hNodeMap, featureName, &hNode);
    CHECK(res);
    if (GENAPIC_INVALID_HANDLE == hNode)
    {
        fprintf(stderr, "There is no feature named '%s'\n", featureName);
        return;
    }

    /* We want a boolean feature node. */
    res = GenApiNodeGetType(hNode, &nodeType);
    CHECK(res);
    if (BooleanNode != nodeType)
    {
        fprintf(stderr, "'%s' is not a boolean feature\n", featureName);
        return;
    }

    /* Check to see if the feature is readable. */
    res = GenApiNodeIsReadable(hNode, &bval);
    CHECK(res);

    if (bval)
    {
        /* Retrieve the current state of the feature. */
        res = GenApiBooleanGetValue(hNode, &value);
        CHECK(res);
        printf("The %s features is %s\n", featureName, value ? "on" : "off");

        /* Set a new value. */
        GenApiNodeIsWritable(hNode, &bval);
        CHECK(res);

        if (bval)
        {
            value = (_Bool) !value;  /* New value */
            printf("Switching the %s feature %s\n", featureName, value ? "on" : "off");
            res = GenApiBooleanSetValue(hNode, value);
            CHECK(res);
        }
        else
            printf("Cannot set value for feature '%s' - node not writable\n", featureName);
    }
    else
        printf("Cannot read feature '%s' - node not readable\n", featureName);
}


/*
  Regardless of the parameter's type, any parameter value can be retrieved as a string. Each parameter
  can be set by passing in a string correspondingly. This function illustrates how to set and get the
  Width parameter as string. As demonstrated above, the Width parameter is of the integer type.
  */
static void
demonstrateFromStringToString(PYLON_DEVICE_HANDLE hDev)
{
    static const char   featureName[] = "Width";   /* The name of the feature. */

    NODEMAP_HANDLE      hNodeMap;
    NODE_HANDLE         hNode;
    EGenApiNodeType     nodeType;
    _Bool                bval;
    GENAPIC_RESULT      res;                       /* Return value. */

    /* Get a handle for the device's node map */
    res = PylonDeviceGetNodeMap(hDev, &hNodeMap);
    CHECK(res);

    /* Look up the feature node. */
    res = GenApiNodeMapGetNode(hNodeMap, featureName, &hNode);
    CHECK(res);
    if (GENAPIC_INVALID_HANDLE == hNode)
    {
        fprintf(stderr, "There is no feature named '%s'\n", featureName);
        return;
    }

    /* We want an integer feature node. */
    res = GenApiNodeGetType(hNode, &nodeType);
    CHECK(res);
    if (IntegerNode != nodeType)
    {
        fprintf(stderr, "'%s' is not an integer feature\n", featureName);
        return;
    }

    /* Check to see if the feature is readable. */
    res = GenApiNodeIsReadable(hNode, &bval);
    CHECK(res);

    if (bval)
    {
        size_t len;
        char   *buf, fixBuf[32];
        _Bool   bval;

        /* Get the value of a feature as a string. Normally, getting the value consists of 3 steps:
           1.) Determine the required buffer size.
           2.) Allocate the buffer.
           3.) Retrieve the value. */
        /* ... Get the required buffer size. The size is queried by
               passing a NULL pointer as a pointer to the buffer. */
        res = GenApiNodeToString(hNode, NULL, &len);
        CHECK(res);
        /* ... len is set to the required buffer size (terminating zero included).
               Allocate the memory and retrieve the string. */
        buf = (char *) malloc(len);
        res = GenApiNodeToString(hNode, buf, &len);
        CHECK(res);

        printf("%s: %s\n", featureName, buf);
        free(buf);

        /* You are not necessarily required to query the buffer size in advance. If the buffer is
           big enough, passing in a buffer and a pointer to its length will work.
           When the buffer is too small, an error is returned. */

        /* Passing in a buffer that is too small. */
        len = 1;
        res = GenApiNodeToString(hNode, fixBuf, &len);
        if (res == GENAPI_E_INSUFFICIENT_BUFFER)
        {
            /* The buffer was too small. The required size is indicated by len. */
            printf("Buffer is too small for the value of '%s'. The required buffer size is %d\n", featureName, (int) len);
        }
        else
            CHECK(res);  /* Unexpected return value. */

        /* Passing in a buffer with sufficient size. */
        len = sizeof fixBuf;
        res = GenApiNodeToString(hNode, fixBuf, &len);
        CHECK(res);


        /* A feature can be set as a string using the GenApiNodeFromString() function.
           If the content of a string can not be converted to the type of the feature, an
           error is returned. */
        GenApiNodeIsWritable(hNode, &bval);
        CHECK(res);

        if (bval)
        {
            res = GenApiNodeFromString(hNode, "fourty-two"); /* Can not be converted to an integer. */
            if (res != GENAPI_E_OK)
            {
                /* Print out an error message. */
                size_t l;
                char *msg;
                GenApiGetLastErrorMessage(NULL, &l); /* Retrieve buffer size for the error message. */
                msg = (char *)malloc(l);             /* Provide memory. */
                GenApiGetLastErrorMessage(msg, &l);  /* Retrieve the message. */
                printf("%s\n", msg);
                free(msg);
            }
        }
        else
            printf("Cannot set value for feature '%s' - node not writable\n", featureName);
    }
    else
        printf("Cannot read feature '%s' - node not readable\n", featureName);
}


/* There are camera features that behave like enumerations. These features can take a value from a fixed
   set of possible values. One example is the pixel format feature. This function illustrates how to deal with
   enumeration features.

*/
static void
demonstrateEnumFeature(PYLON_DEVICE_HANDLE hDev)
{
    static const char   featureName[] = "PixelFormat";
    NODEMAP_HANDLE      hNodeMap;
    NODE_HANDLE         hNode;
    EGenApiNodeType     nodeType;
    _Bool                bval;
    GENAPIC_RESULT      res;                           /* Return value. */


    /* Get a handle for the device's node map. */
    res = PylonDeviceGetNodeMap(hDev, &hNodeMap);
    CHECK(res);

    /* Look up the feature node. */
    res = GenApiNodeMapGetNode(hNodeMap, featureName, &hNode);
    CHECK(res);
    if (GENAPIC_INVALID_HANDLE == hNode)
    {
        fprintf(stderr, "There is no feature named '%s'\n", featureName);
        return;
    }

    /* We want an enumeration feature node. */
    res = GenApiNodeGetType(hNode, &nodeType);
    CHECK(res);
    if (EnumerationNode != nodeType)
    {
        fprintf(stderr, "'%s' is not an enumeration feature\n", featureName);
        return;
    }

    /* Check to see if the feature is readable. */
    res = GenApiNodeIsReadable(hNode, &bval);
    CHECK(res);

    /* The allowed values for an enumeration feature are represented as strings. Use the
    GenApiNodeFromString() and GenApiNodeToString() methods for setting and getting
    the value of an enumeration feature. */

    if (bval)
    {
        /* Symbolic names of pixel formats. */
        static const char   symMono8[] = "Mono8",
                            symMono16[] = "Mono16",
                            symYUV422Packed[] = "YUV422Packed";

        size_t      len;                           /* The length of the string. */
        char        value[64];                     /* The current value of the feature. */
        _Bool       supportsMono8, supportsYUV422Packed, supportsMono16;
        NODE_HANDLE hEntry;


        /* Get the current value of the enumeration feature. */
        len = sizeof value;
        res = GenApiNodeToString(hNode, value, &len);
        CHECK(res);

        printf("PixelFormat: %s\n", value);

        /*
        For an enumeration feature, the pylon Viewer's "Feature Documentation" window lists the
        names of the possible values. Some of the values may not be supported by the device.
        To check if a certain "SomeValue" value for a "SomeFeature" feature can be set, call the
        GenApiNodeIsAvailable() on the node of the enum entry.
        */

        /* Check to see if the Mono8 pixel format can be set. */
        res = GenApiEnumerationGetEntryByName(hNode, symMono8, &hEntry);
        CHECK(res);
        if ( hEntry != GENAPIC_INVALID_HANDLE )
        {
            res = GenApiNodeIsAvailable(hEntry, &supportsMono8);
            CHECK(res);
        }
        else
        {
            supportsMono8 = 0;
        }
        printf("%s %s a supported value for the PixelFormat feature\n", symMono8, supportsMono8 ? "is" : "is not");

        /* Check to see if the YUV422Packed pixel format can be set. */
        res = GenApiEnumerationGetEntryByName(hNode, symYUV422Packed, &hEntry);
        CHECK(res);
        if ( hEntry != GENAPIC_INVALID_HANDLE )
        {
            res = GenApiNodeIsAvailable(hEntry, &supportsYUV422Packed);
            CHECK(res);
        }
        else
        {
            supportsYUV422Packed = 0;
        }
        printf("%s %s a supported value for the PixelFormat feature\n", symYUV422Packed, supportsYUV422Packed ? "is" : "is not");

        /* Check to see if the Mono16 pixel format can be set. */
        res = GenApiEnumerationGetEntryByName(hNode, symMono16, &hEntry);
        CHECK(res);
        if ( hEntry != GENAPIC_INVALID_HANDLE )
        {
            res = GenApiNodeIsAvailable(hEntry, &supportsMono16);
            CHECK(res);
        }
        else
        {
            supportsMono16 = 0;
        }
        printf("%s %s a supported value for the PixelFormat feature\n", symMono16, supportsMono16 ? "is" : "is not");


        /* Before writing a value, we recommend checking if the enumeration feature is
        currently writable. */
        res = GenApiNodeIsWritable(hNode, &bval);
        CHECK(res);

        if (bval)
        {
            /* The PixelFormat feature is writable, set it to one of the supported values. */
            if (supportsMono16)
            {
                printf("Setting PixelFormat to Mono16\n");
                res = GenApiNodeFromString(hNode, symMono16);
                CHECK(res);
            }
            else if (supportsYUV422Packed)
            {
                printf("Setting PixelFormat to YUV422Packed\n");
                res = GenApiNodeFromString(hNode, symYUV422Packed);
                CHECK(res);
            }
            else if (supportsMono8)
            {
                printf("Setting PixelFormat to Mono8\n");
                res = GenApiNodeFromString(hNode, symMono8);
                CHECK(res);
            }

            /* Reset the PixelFormat feature to its previous value. */
            res = GenApiNodeFromString(hNode, value);
            CHECK(res);
        }
        else
            printf("Cannot set value for feature '%s' - node not writable\n", featureName);
    }
    else
        printf("Cannot read feature '%s' - node not readable\n", featureName);
}


/* Enumerate all possible entries for an enumerated feature. For every entry, a selection
   of properties is displayed. A loop similar to the one shown below may be part of a
   GUI program that wants to fill the entries of a menu. */
static void
demonstrateEnumIteration(PYLON_DEVICE_HANDLE hDev)
{
    static const char   featureName[] = "PixelFormat";
    NODEMAP_HANDLE      hNodeMap;
    NODE_HANDLE         hNode;
    EGenApiNodeType     nodeType;
    _Bool                bval;
    GENAPIC_RESULT      res;                           /* Return value. */


    /* Get a handle for the device's node map */
    res = PylonDeviceGetNodeMap(hDev, &hNodeMap);
    CHECK(res);

    /* Look up the feature node. */
    res = GenApiNodeMapGetNode(hNodeMap, featureName, &hNode);
    CHECK(res);
    if (GENAPIC_INVALID_HANDLE == hNode)
    {
        fprintf(stderr, "There is no feature named '%s'\n", featureName);
        return;
    }

    /* We want an enumeration feature node. */
    res = GenApiNodeGetType(hNode, &nodeType);
    CHECK(res);
    if (EnumerationNode != nodeType)
    {
        fprintf(stderr, "'%s' is not an enumeration feature\n", featureName);
        return;
    }

    /* Check to see if the feature is readable. */
    res = GenApiNodeIsReadable(hNode, &bval);
    CHECK(res);

    if (bval)
    {
        size_t max, i;

        /* check entries. */
        res = GenApiEnumerationGetNumEntries(hNode, &max);
        CHECK(res);

        /* Write out header. */
        printf("Allowed values for feature '%s':\n"
               "--------------\n",
               featureName);

        /* A loop to visit every enumeration entry node once. */
        for (i = 0; i < max; i++)
        {
            NODE_HANDLE hEntry;
            char name[128], displayName[STRING_BUFFER_SIZE], description[STRING_BUFFER_SIZE];
            size_t siz;
            _Bool avail;

            /* Get handle for enumeration entry node. */
            res = GenApiEnumerationGetEntryByIndex(hNode, i, &hEntry);
            CHECK(res);

            /* Get node name. */
            siz = sizeof name;
            res = GenApiNodeGetName(hEntry, name, &siz);
            CHECK(res);

            /* Get display name. */
            siz = sizeof displayName;
            res = GenApiNodeGetDisplayName(hEntry, displayName, &siz);
            CHECK(res);

            /* Get description. */
            siz = sizeof description;
            res = GenApiNodeGetDescription(hEntry, description, &siz);
            CHECK(res);

            /* Get availability. */
            res = GenApiNodeIsAvailable(hEntry, &avail);
            CHECK(res);

            /* Write out results. */
            printf("Node name:    %s\n"
                   "Display name: %s\n"
                   "Description:  %s\n"
                   "Available:    %s\n"
                   "--------------\n",
                   name, displayName, description, avail ? "yes" : "no");
        }
    }
    else
        printf("Cannot read feature '%s' - node not readable\n", featureName);
}



/* Traverse the feature tree, displaying all categories and all features. */
static void
handleCategory(NODE_HANDLE hRoot, char * buf, unsigned int depth)
{
    GENAPIC_RESULT      res;
    size_t              bufsiz, siz, numfeat, i;

    /* Write out node name. */
    siz = bufsiz = STRING_BUFFER_SIZE - depth * 2;
    res = GenApiNodeGetName(hRoot, buf, &siz);
    CHECK(res);

    /* Get the number of feature nodes in this category. */
    res = GenApiCategoryGetNumFeatures(hRoot, &numfeat);
    CHECK(res);

    printf("%s category has %u children\n", buf - depth * 2, (unsigned int) numfeat);


    /* Increase indentation. */
    *buf++ = ' ';
    *buf++ = ' ';
    bufsiz -= 2;
    ++depth;

    /* Now loop over all feature nodes. */
    for (i = 0; i < numfeat; ++i)
    {
        NODE_HANDLE         hNode;
        EGenApiNodeType     nodeType;

        /* Get next feature node and check its type. */
        res = GenApiCategoryGetFeatureByIndex(hRoot, i, &hNode);
        CHECK(res);
        res = GenApiNodeGetType(hNode, &nodeType);
        CHECK(res);

        if (Category != nodeType)
        {
            /* A regular feature. */
            EGenApiAccessMode am;
            const char *amode;

            siz = bufsiz;
            res = GenApiNodeGetName(hNode, buf, &siz);
            CHECK(res);
            res = GenApiNodeGetAccessMode(hNode, &am);
            CHECK(res);

            switch (am)
            {
            case NI:
                amode = "not implemented";
                break;
            case NA:
                amode = "not available";
                break;
            case WO:
                amode = "write only";
                break;
            case RO:
                amode = "read only";
                break;
            case RW:
                amode = "read and write";
                break;
            default:
                amode = "undefined";
                break;
            }

            printf("%s feature - access: %s\n", buf - depth * 2, amode);
        }
        else
            /* Another category node. */
            handleCategory(hNode, buf, depth);
    }
}

static void
demonstrateCategory(PYLON_DEVICE_HANDLE hDev)
{
    NODEMAP_HANDLE      hNodeMap;
    NODE_HANDLE         hNode;
    char                buf[512];
    GENAPIC_RESULT      res;

    /* Get a handle for the device's node map. */
    res = PylonDeviceGetNodeMap(hDev, &hNodeMap);
    CHECK(res);

    /* Look up the root node. */
    res = GenApiNodeMapGetNode(hNodeMap, "Root", &hNode);
    CHECK(res);

    handleCategory(hNode, buf, 0);
}



/* There are camera features, such as starting image acquisition, that represent a command.
   This function that loads the factory settings, illustrates how to execute a command feature.  */
static void
demonstrateCommandFeature(PYLON_DEVICE_HANDLE hDev)
{
    static const char   selectorName[] = "UserSetSelector",
                        commandName[] = "UserSetLoad";
    NODEMAP_HANDLE      hNodeMap;
    NODE_HANDLE         hCommand, hSelector;
    EGenApiNodeType     nodeType;
    _Bool                bval;
    GENAPIC_RESULT      res;  /* Return value. */

    /* Get a handle for the device's node map. */
    res = PylonDeviceGetNodeMap(hDev, &hNodeMap);
    CHECK(res);

    /* Look up the command node. */
    res = GenApiNodeMapGetNode(hNodeMap, commandName, &hCommand);
    CHECK(res);
    if (GENAPIC_INVALID_HANDLE == hCommand)
    {
        fprintf(stderr, "There is no feature named '%s'\n", commandName);
        return;
    }

    /* Look up the selector node. */
    res = GenApiNodeMapGetNode(hNodeMap, selectorName, &hSelector);
    CHECK(res);
    if (GENAPIC_INVALID_HANDLE == hSelector)
    {
        fprintf(stderr, "There is no feature named '%s'\n", selectorName);
        return;
    }

    /* We want a command feature node. */
    res = GenApiNodeGetType(hCommand, &nodeType);
    CHECK(res);
    if (CommandNode != nodeType)
    {
        fprintf(stderr, "'%s' is not a command feature\n", selectorName);
        return;
    }

    /* Before executing the user set load command, the configuration set selector must be
       set to the default set. */

    /* Check to see if the selector is writable. */
    res = GenApiNodeIsWritable(hSelector, &bval);
    CHECK(res);

    if (bval)
    {
        /* Choose the default configuration set (with one of the factory setups chosen). */
        res = GenApiNodeFromString(hSelector, "Default");
        CHECK(res);
    }
    else
        printf("Cannot set selector '%s' - node not writable\n", selectorName);


    /* Check to see if the command is writable. */
    res = GenApiNodeIsWritable(hCommand, &bval);
    CHECK(res);

    if (bval)
    {
        /* Execute the configuration set load command. */
        printf("Loading the default set.\n");
        res = GenApiCommandExecute(hCommand);
        CHECK(res);
    }
    else
        printf("Cannot execute command '%s' - node not writable\n", commandName);
}


int
main(void)
{
    GENAPIC_RESULT              res;           /* Return value of pylon methods. */
    size_t                      numDevices;    /* Number of available devices. */
    PYLON_DEVICE_HANDLE         hDev;          /* Handle for the pylon device. */

    /* Before using any pylon methods, the pylon runtime must be initialized. */
    PylonInitialize();

    /* Enumerate all camera devices. You must call
    PylonEnumerateDevices() before creating a device. */
    res = PylonEnumerateDevices(&numDevices);
    CHECK(res);
    if (0 == numDevices)
    {
        fprintf(stderr, "No devices found.\n");
        PylonTerminate();
        pressEnterToExit();
        exit(EXIT_FAILURE);
    }

    /* Get a handle for the first device found.  */
    res = PylonCreateDeviceByIndex(0, &hDev);
    CHECK(res);

    /* Before using the device, it must be opened. Open it for configuring
    parameters and for grabbing images. */
    res = PylonDeviceOpen(hDev, PYLONC_ACCESS_MODE_CONTROL | PYLONC_ACCESS_MODE_STREAM);
    CHECK(res);

    /* Print out the name of the camera we are using. */
    {
        char buf[256];
        size_t siz = sizeof(buf);
        _Bool isReadable;

        isReadable = PylonDeviceFeatureIsReadable(hDev, "DeviceModelName");
        if (isReadable)
        {
            res = PylonDeviceFeatureToString(hDev, "DeviceModelName", buf, &siz);
            CHECK(res);
            printf("Using camera %s\n", buf);
        }
    }

    /* Demonstrate how to check the accessibility of a feature. */
    demonstrateAccessibilityCheck(hDev);
    puts("");

    /* Demonstrate how to handle integer camera parameters. */
    demonstrateIntFeature(hDev);
    puts("");

    /* Demonstrate how to handle floating point camera parameters. */
    demonstrateFloatFeature(hDev);
    puts("");

    /* Demonstrate how to handle boolean camera parameters. */
    demonstrateBooleanFeature(hDev);
    puts("");

    /* Each feature can be read as a string and also set as a string. */
    demonstrateFromStringToString(hDev);
    puts("");

    /* Demonstrate how to handle enumeration camera parameters. */
    demonstrateEnumFeature(hDev);
    puts("");

    /* Demonstrate how to iterate enumeration entries. */
    demonstrateEnumIteration(hDev);
    puts("");

    /* Demonstrate how to execute actions. */
    demonstrateCommandFeature(hDev);
    puts("");

    /* Demonstrate category nodes. */
    demonstrateCategory(hDev);


    /* Clean up. Close and release the pylon device. */

    res = PylonDeviceClose(hDev);
    CHECK(res);
    res = PylonDestroyDevice (hDev);
    CHECK(res);


    /* Shut down the pylon runtime system. Don't call any pylon method after
       calling PylonTerminate(). */
    PylonTerminate();
    pressEnterToExit();

    return EXIT_SUCCESS;
}

/* This function can be used to wait for user input at the end of the sample program. */
void pressEnterToExit(void)
{
    fprintf( stderr, "\nPress enter to exit.\n");
    while( getchar() != '\n');
}

