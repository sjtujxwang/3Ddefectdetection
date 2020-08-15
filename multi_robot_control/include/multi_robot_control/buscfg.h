#pragma once

#if defined _WIN32
#   if defined BUS_STATIC
#       define BUS_EXPORT
#   elif defined BUS_DLL
#       define BUS_EXPORT __declspec(dllexport)
#   else
#       define BUS_EXPORT __declspec(dllimport)
#   endif
#else
#   define BUS_EXPORT
#endif

#ifdef __cplusplus
#define BUS_API	extern "C"
#else
#define BUS_API	extern
#endif // __cplusplus

#ifdef __cplusplus
#ifndef BUS_NS
#define	BUS_NS			bus

#define BUS_NS_BEGIN	namespace bus{
#define	BUS_NS_END		}

#define USING_BUS_NS	using namespace bus;
#endif
#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////
// API Version
#define		LIBBUS_VERSION		"1.0.0.0"
