#ifndef __TYPE_ID_H__
#define __TYPE_ID_H__

#include <typeinfo>

#define DEFINE_TYPE_ID(typeName) \
    static inline std::size_t ID() \
    { \
        static const std::size_t id = typeid(typeName).hash_code(); \
        return id; \
    } \
    inline std::size_t GetClassID() const override \
    { \
        return ID(); \
    } \
    inline std::string GetClassName() const override \
    { \
        return typeid(typeName).name(); \
    }

#endif // __TYPE_ID_H__
