
#ifndef SPKT_FACTORY_INFO_H
#define SPKT_FACTORY_INFO_H

#include <type_traits>
#include <map>
#include <sprockit/sim_parameters_fwd.h>
#include <sprockit/errors.h>

namespace sprockit {

template <class Base, class... Args>
struct Builder
{
  typedef Base* (*createFxn)(Args...);

  virtual Base* create(Args... ctorArgs) = 0;

  virtual ~Builder(){}
};

template <class Base, class... CtorArgs>
class BuilderLibrary
{
 public:
  using BaseBuilder = Builder<Base,CtorArgs...>;

  BaseBuilder* getBuilder(const std::string &name) {
    auto iter = factories_.find(name);
    if (iter == factories_.end()){
      return nullptr;
    } else {
      return iter->second.get();
    }
  }

  const std::map<std::string, std::unique_ptr<BaseBuilder>>& getMap() const {
    return factories_;
  }

  bool addBuilder(const std::string& name, std::unique_ptr<BaseBuilder>&& fact){
    factories_[name] = std::move(fact);
    return true;
  }

 private:
  std::map<std::string, std::unique_ptr<BaseBuilder>> factories_;
};

template <class Base, class... CtorArgs>
class BuilderLibraryDatabase {
 public:
  using Library=BuilderLibrary<Base,CtorArgs...>;
  using LibraryPtr=std::unique_ptr<Library>;
  using BaseFactory=typename Library::BaseBuilder;

  static Library* getLibrary(const std::string& name){
    if (!libraries){
      libraries = std::unique_ptr<std::map<std::string,LibraryPtr>>(new std::map<std::string,LibraryPtr>);
    }
    auto iter = libraries->find(name);
    if (iter == libraries->end()){
      auto info = std::unique_ptr<Library>(new Library);
      auto* ptr = info.get();
      (*libraries)[name] = std::move(info);
      return ptr;
    } else {
      return iter->second.get();
    }
  }

 private:
  // Database - needs to be a pointer for static init order
  static std::unique_ptr<std::map<std::string,std::unique_ptr<Library>>> libraries;
};

template <class Base, class... CtorArgs>
 std::unique_ptr<std::map<std::string,std::unique_ptr<BuilderLibrary<Base,CtorArgs...>>>>
  BuilderLibraryDatabase<Base,CtorArgs...>::libraries;


template <class Base, class T>
struct InstantiateBuilder {
  static bool isLoaded() {
    return loaded;
  }

  static const bool loaded;
};

template <class Base, class T>
 const bool InstantiateBuilder<Base,T>::loaded = Base::Ctor::template add<T>(T::SPKT_getLibrary(), T::SPKT_getName());

template <class Base, class T, class Enable=void>
struct Allocator
{
  template <class... Args>
  T* operator()(Args&&... args){
    return new T(std::forward<Args>(args)...);
  }
};

template <class T> struct wrap { using type=void; };
template <class Base, class T>
struct Allocator<Base,T,
  typename wrap<decltype(std::declval<T>().finalizeInit(std::declval<SST::Params&>()))>::type>
{
  template <class... Args>
  Base* operator()(SST::Params& params, Args&&... args){
    T* t = new T(params, std::forward<Args>(args)...);
    t->finalizeInit(params);
    return t;
  }
};

template <class Base, class T>
struct CachedAllocator
{
  template <class... Args>
  Base* operator()(Args&&... ctorArgs) {
    if (!cached_){
      cached_ = new T(std::forward<Args>(ctorArgs)...);
    }
    return cached_;
  }

  static Base* cached_;
};
template <class Base, class T>
  Base* CachedAllocator<Base,T>::cached_ = nullptr;

template <class Base, class T, class... Args>
struct DerivedBuilder : public Builder<Base,Args...>
{
  Base* create(Args... ctorArgs) override {
    return Allocator<Base,T>()(std::forward<Args>(ctorArgs)...);
  }
};


template <class T, class U>
struct is_tuple_constructible : public std::false_type {};

template <class T, class... Args>
struct is_tuple_constructible<T, std::tuple<Args...>> :
  public std::is_constructible<T, Args...>
{
};

struct BuilderDatabase {
  template <class T, class... Args>
  static BuilderLibrary<T,Args...>* getLibrary(const std::string& name){
    return BuilderLibraryDatabase<T,Args...>::getLibrary(name);
  }
};

template <class Base, class CtorTuple>
struct ElementsBuilder {};

template <class Base, class... Args>
struct ElementsBuilder<Base, std::tuple<Args...>>
{
  static BuilderLibrary<Base,Args...>* getLibrary(const std::string& name){
    return BuilderLibraryDatabase<Base,Args...>::getLibrary(name);
  }

  template <class T> static Builder<Base,Args...>* makeBuilder(){
    return new DerivedBuilder<Base,T,Args...>();
  }

};

template <class Base, class T, class... Args>
std::unique_ptr<DerivedBuilder<Base,T,Args...>>
makeDerivedBuilder(){
  using ret = DerivedBuilder<Base,T,Args...>;
  return std::unique_ptr<ret>(new ret);
}

template <class Base, class... Args>
struct SingleCtor
{
  template <class T> static bool add(const std::string& elemlib, const std::string& elem){
    //if abstract, force an allocation to generate meaningful errors
    Base::addBuilder(elemlib,elem,makeDerivedBuilder<Base,T,Args...>());
    return true;
  }
};


template <class Base, class Ctor, class... Ctors>
struct CtorList : public CtorList<Base,Ctors...>
{
  template <class T, int NumValid=0, class U=T>
  static typename std::enable_if<std::is_abstract<U>::value || is_tuple_constructible<U,Ctor>::value, bool>::type
  add(const std::string& elemlib, const std::string& elem){
    //if abstract, force an allocation to generate meaningful errors
    auto fact = ElementsBuilder<Base,Ctor>::template makeBuilder<U>();
    Base::addBuilder(elemlib,elem,std::move(fact));
    return CtorList<Base,Ctors...>::template add<T,NumValid+1>(elemlib,elem);
  }

  template <class T, int NumValid=0, class U=T>
  static typename std::enable_if<!std::is_abstract<U>::value && !is_tuple_constructible<U,Ctor>::value, bool>::type
  add(const std::string& elemlib, const std::string& elem){
    return CtorList<Base,Ctors...>::template add<T,NumValid>(elemlib,elem);
  }

};

template <int NumValid>
struct NoValidConstructorsForDerivedType {
  static constexpr bool atLeastOneValidCtor = true;
};

template <> class NoValidConstructorsForDerivedType<0> {};

template <class Base> struct CtorList<Base,void>
{
  template <class T,int numValidCtors>
  static bool add(const std::string& lib, const std::string& elem){
    return NoValidConstructorsForDerivedType<numValidCtors>::atLeastOneValidCtor;
  }
};

template <class Base, class Enable=void>
struct GetBaseName {};

template <class Base>
struct GetBaseName<Base, typename wrap<decltype(Base::ELI_baseName())>::type>
{
  const char* operator()(){
    return Base::ELI_baseName();
  }
};

template <class Base>
struct GetBaseName<Base, typename wrap<decltype(Base::SPKT_baseName())>::type>
{
  const char* operator()(){
    return Base::SPKT_baseName();
  }
};



template <class Base, class... Args>
Base* create(const std::string& elemlib, const std::string& elem, Args&&... args){
  auto* lib = Base::getBuilderLibrary(elemlib);
  if (!lib){
    spkt_abort_printf("cannot find library '%s' for base type '%s'",
                      elemlib.c_str(), GetBaseName<Base>()());
  }
  auto* builder = lib->getBuilder(elem);
  if (!builder){
    spkt_abort_printf("cannot find builder '%s' in library '%s' for base type %s",
                      elem.c_str(), elemlib.c_str(), GetBaseName<Base>()());
  }
  return builder->create(std::forward<Args>(args)...);
}

}

#define SPKT_CTOR(...) std::tuple<__VA_ARGS__>
#define SPKT_DEFAULT_CTOR() std::tuple<>
#define SPKT_FORWARD_AS_ONE(...) __VA_ARGS__

#define SPKT_DECLARE_BASE(Base) \
  using __LocalSpktBase = Base; \
  static const char* SPKT_baseName(){ return #Base; }

#define SPKT_CTORS_COMMON(...) \
  using Ctor = sprockit::CtorList<__LocalSpktBase,__VA_ARGS__,void>; \
  template <class __TT, class... __CtorArgs> \
  using DerivedBuilder = sprockit::DerivedBuilder<__LocalSpktBase,__TT,__CtorArgs...>; \
  template <class... __InArgs> static sprockit::BuilderLibrary<__LocalSpktBase,__InArgs...>* \
  getBuilderLibraryTemplate(const std::string& name){ \
    return sprockit::BuilderDatabase::getLibrary<__LocalSpktBase,__InArgs...>(name); \
  } \
  template <class __TT> static bool addDerivedBuilder(const std::string& lib, const std::string& elem){ \
    return Ctor::template add<0,__TT>(lib,elem); \
  } \

#define SPKT_DECLARE_CTORS(...) \
  SPKT__CTORS_COMMON(SPKT_FORWARD_AS_ONE(__VA_ARGS__)) \
  template <class... Args> static bool addBuilder(const std::string& elemlib, const std::string& elem, \
                                           sprockit::Builder<__LocalSpktBase,Args...>* builder){ \
    return getBuilderLibraryTemplate<Args...>(elemlib)->addBuilder(elem, builder); \
  }

#define SPKT_DECLARE_CTORS_EXTERN(...) \
  SPKT_CTORS_COMMON(SPKT_FORWARD_AS_ONE(__VA_ARGS__))

#define SPKT_CTOR_COMMON(...) \
  using Ctor = sprockit::SingleCtor<__LocalSpktBase,__VA_ARGS__>; \
  using BaseBuilder = sprockit::Builder<__LocalSpktBase,__VA_ARGS__>; \
  using BuilderLibrary = sprockit::BuilderLibrary<__LocalSpktBase,__VA_ARGS__>; \
  using BuilderLibraryDatabase = sprockit::BuilderLibraryDatabase<__LocalSpktBase,__VA_ARGS__>; \
  template <class __TT> using DerivedBuilder = sprockit::DerivedBuilder<__LocalSpktBase,__TT,__VA_ARGS__>; \
  template <class __TT> static bool addDerivedBuilder(const std::string& lib, const std::string& elem){ \
    return addBuilder(lib,elem,new DerivedBuilder<__TT>); \
  }

//I can make some extra using typedefs because I have only a single ctor
#define SPKT_DECLARE_CTOR(...) \
  SPKT_CTOR_COMMON(__VA_ARGS__) \
  static BuilderLibrary* getBuilderLibrary(const std::string& name){ \
    return sprockit::BuilderDatabase::getLibrary<__LocalSpktBase,__VA_ARGS__>(name); \
  } \
  static bool addBuilder(const std::string& elemlib, const std::string& elem, \
                std::unique_ptr<BaseBuilder>&& builder){ \
    return getBuilderLibrary(elemlib)->addBuilder(elem,std::move(builder)); \
  }

#define SPKT_DECLARE_CTOR_EXTERN(...) \
  SPKT_CTOR_COMMON(__VA_ARGS__) \
  static BuilderLibrary* getBuilderLibrary(const std::string& name); \
  static bool addBuilder(const std::string& elemlib, const std::string& elem, \
                         std::unique_ptr<BaseBuilder>&& builder);

#define SPKT_DEFINE_CTOR_EXTERN(base) \
  bool base::addBuilder(const std::string& elemlib, const std::string& elem, \
                        std::unique_ptr<BaseBuilder>&& builder){ \
    return getBuilderLibrary(elemlib)->addBuilder(elem,std::move(builder)); \
  } \
  base::BuilderLibrary* base::getBuilderLibrary(const std::string& elemlib){ \
    return BuilderLibraryDatabase::getLibrary(elemlib); \
  }

#define SPKT_DEFAULT_CTOR_COMMON() \
  using Ctor = sprockit::SingleCtor<__LocalSpktBase>; \
  using BaseBuilder = sprockit::Builder<__LocalSpktBase>; \
  using BuilderLibrary = sprockit::BuilderLibrary<__LocalSpktBase>; \
  template <class __TT> using DerivedBuilder = sprockit::DerivedBuilder<__LocalSpktBase,__TT>;

//I can make some extra using typedefs because I have only a single ctor
#define SPKT_DECLARE_DEFAULT_CTOR() \
  SPKT_DEFAULT_CTOR_COMMON() \
  static BuilderLibrary* getBuilderLibrary(const std::string& name){ \
    return sprockit::BuilderDatabase::getLibrary<__LocalSpktBase>(name); \
  } \
  static bool addBuilder(const std::string& elemlib, const std::string& elem, \
                         std::unique_ptr<BaseBuilder>&& builder){ \
    return getBuilderLibrary(elemlib)->addBuilder(elem,std::move(builder)); \
  }

#define SPKT_DECLARE_DEFAULT_CTOR_EXTERN() \
  SPKT_DEFAULT_CTOR_COMMON() \
  static BuilderLibrary* getBuilderLibrary(const std::string& name); \
  static bool addBuilder(const std::string& elemlib, const std::string& elem, \
                         std::unique_ptr<BaseBuilder>&& builder);

#define SPKT_REGISTER_DERIVED(base,cls,lib,name,desc) \
  bool ELI_isLoaded() { \
    return sprockit::InstantiateBuilder<base,cls>::isLoaded(); \
  } \
    static const std::string SPKT_getLibrary() { \
    return lib; \
  } \
  static const std::string SPKT_getName() { \
    return name; \
  } 


#endif

