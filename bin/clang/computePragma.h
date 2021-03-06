/**
Copyright 2009-2018 National Technology and Engineering Solutions of Sandia, 
LLC (NTESS).  Under the terms of Contract DE-NA-0003525, the U.S.  Government 
retains certain rights in this software.

Sandia National Laboratories is a multimission laboratory managed and operated
by National Technology and Engineering Solutions of Sandia, LLC., a wholly 
owned subsidiary of Honeywell International, Inc., for the U.S. Department of 
Energy's National Nuclear Security Administration under contract DE-NA0003525.

Copyright (c) 2009-2018, NTESS

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Questions? Contact sst-macro-help@sandia.gov
*/
#ifndef bin_clang_computePragma_h
#define bin_clang_computePragma_h

#include "pragmas.h"

class SSTComputePragma : public SSTPragma {
  friend class ComputeVisitor;
 public:
  SSTComputePragma() : SSTPragma(Compute){}

  SSTComputePragma(const std::string& nthread) :
    SSTPragma(Compute), nthread_(nthread){}

  static void replaceForStmt(clang::ForStmt* stmt, clang::CompilerInstance& CI,
                             SSTPragmaList& prgList, clang::Rewriter& r,
                             PragmaConfig& cfg, SkeletonASTVisitor* visitor,
                             const std::string& nthread);

 private:
  void activate(clang::Stmt *stmt, clang::Rewriter &r, PragmaConfig& cfg) override;
  void activate(clang::Decl* decl, clang::Rewriter& r, PragmaConfig& cfg) override;
  void defaultAct(clang::Stmt* stmt, clang::Rewriter &r, PragmaConfig& cfg);
  void visitForStmt(clang::ForStmt* stmt, clang::Rewriter& r, PragmaConfig& cfg);
  void visitCXXMethodDecl(clang::CXXMethodDecl* decl, clang::Rewriter& r, PragmaConfig& cfg);
  void visitFunctionDecl(clang::FunctionDecl* decl, clang::Rewriter& r, PragmaConfig& cfg);
  void visitIfStmt(clang::IfStmt* stmt, clang::Rewriter& r, PragmaConfig& cfg);
  void visitAndReplaceStmt(clang::Stmt* stmt, clang::Rewriter& r, PragmaConfig& cfg);

  std::string nthread_;

 protected:
  SSTComputePragma(SSTPragma::class_t cls) : SSTPragma(cls) {}
};

class SSTAlwaysComputePragma : public SSTComputePragma
{
 public:
  SSTAlwaysComputePragma() : SSTComputePragma(AlwaysCompute) {}
};

class SSTMemoryPragma : public SSTPragma {
 public:
  SSTMemoryPragma(const std::string& memSpec) : SSTPragma(Memory), memSpec_(memSpec){}
 private:
  void activate(clang::Stmt *s, clang::Rewriter &r, PragmaConfig &cfg);
  std::string memSpec_;
};

class SSTLoopCountPragma : public SSTPragma {
 public:
  SSTLoopCountPragma(const std::list<clang::Token> &tokens);

  const std::string& count() const {
    return loopCount_;
  }

  void activate(clang::Stmt *s, clang::Rewriter &r, PragmaConfig &cfg) override {
    //no activate actions are actually required
    //other code in computeVisitor.cc will detect this pragma and trigger actions
  }

  bool reusable() const override {
    return true;
  }

 private:
  std::string loopCount_;
};

class SSTMemoizeComputePragma : public SSTPragma
{
 public:
  SSTMemoizeComputePragma(const std::string& token,
      bool skeletonize, const std::string& model,
      std::list<std::string>&& inputs,
      bool givenName) :
    SSTPragma(Memoize),
    token_(token),
    skeletonize_(skeletonize),
    model_(model),
    givenName_(givenName),
    inputs_(std::move(inputs))
  {}

  bool firstPass(const clang::Decl *d) const override {
    return true;
  }

  void activate(clang::Stmt *s, clang::Rewriter &r, PragmaConfig &cfg) override;
  void activate(clang::Decl *d, clang::Rewriter &r, PragmaConfig &cfg) override;

 private:
  void doReplace(clang::SourceLocation startInsert, clang::SourceLocation finalInsert, clang::Stmt* stmt,
                 bool insertStartAfter, bool insertFinalAfter,
                 clang::Rewriter& r, clang::Expr** callArgs, const clang::ParmVarDecl** callParams);

  std::string token_;
  std::string model_;
  bool skeletonize_;
  std::list<std::string> inputs_;
  bool givenName_;
  std::list<int> fxnArgInputs_;
  std::set<clang::Decl*> written_;
};

class SSTImplicitStatePragma : public SSTPragma
{
 public:
  SSTImplicitStatePragma(std::map<std::string,std::string>&& values) :
    SSTPragma(ImplicitState),
    values_(std::move(values))
  {}

  bool firstPass(const clang::Decl *d) const override {
    return true;
  }

  void activate(clang::Stmt *s, clang::Rewriter &r, PragmaConfig &cfg) override;
  void activate(clang::Decl *d, clang::Rewriter &r, PragmaConfig &cfg) override;

 private:
  void doReplace(clang::SourceLocation startInsert, clang::SourceLocation finalInsert,
                 bool insertStartAfter, bool insertFinalAfter,
                 clang::Rewriter& r, const std::map<std::string,std::string>& values);

  std::map<std::string,int> fxnArgValues_;
  std::map<std::string,std::string> values_;
  std::set<clang::Decl*> written_;
};

class SSTOpenMPParallelPragmaHandler : public SSTPragmaHandler
{
 public:
  SSTOpenMPParallelPragmaHandler(SSTPragmaList& plist,
                         clang::CompilerInstance& CI,
                         SkeletonASTVisitor& visitor) :
      SSTPragmaHandler("parallel", plist, CI, visitor){}
 private:
  SSTPragma* handleSSTPragma(const std::list<clang::Token> &tokens) const override;
};

class SSTLoopCountPragmaHandler : public SSTPragmaHandler
{
 public:
  SSTLoopCountPragmaHandler(SSTPragmaList& plist,
                        clang::CompilerInstance& CI,
                        SkeletonASTVisitor& visitor) :
     SSTPragmaHandler("loop_count", plist, CI, visitor){}
 private:
  SSTPragma* handleSSTPragma(const std::list<clang::Token> &tokens) const override {
    //this actually just maps cleanly into a compute pragma
    return new SSTLoopCountPragma(tokens);
  }
};

class SSTMemoryPragmaHandler : public SSTPragmaHandler
{
 public:
  SSTMemoryPragmaHandler(SSTPragmaList& plist,
                        clang::CompilerInstance& CI,
                        SkeletonASTVisitor& visitor) :
     SSTPragmaHandler("memory", plist, CI, visitor){}
 private:
  SSTPragma* handleSSTPragma(const std::list<clang::Token> &tokens) const override;
};

class SSTComputePragmaHandler : public SSTSimplePragmaHandler<SSTComputePragma> {
 public:
  SSTComputePragmaHandler(SSTPragmaList& plist, clang::CompilerInstance& CI,
                      SkeletonASTVisitor& visitor) :
   SSTSimplePragmaHandler<SSTComputePragma>("compute", plist, CI, visitor)
  {}
};

class SSTAlwaysComputePragmaHandler : public SSTSimplePragmaHandler<SSTAlwaysComputePragma> {
 public:
  SSTAlwaysComputePragmaHandler(SSTPragmaList& plist, clang::CompilerInstance& CI,
                      SkeletonASTVisitor& visitor) :
   SSTSimplePragmaHandler<SSTAlwaysComputePragma>("always_compute", plist, CI, visitor)
  {}
};

class SSTMemoizeComputePragmaHandler : public SSTStringMapPragmaHandler
{
 public:
  SSTMemoizeComputePragmaHandler(SSTPragmaList& plist, clang::CompilerInstance& CI,
                     SkeletonASTVisitor& visitor) :
   SSTStringMapPragmaHandler("memoize", plist, CI, visitor)
  {}
 private:
  SSTPragma* allocatePragma(const std::map<std::string, std::list<std::string>>& args) const override;
};

class SSTImplicitStatePragmaHandler : public SSTStringMapPragmaHandler
{
 public:
  SSTImplicitStatePragmaHandler(SSTPragmaList& plist, clang::CompilerInstance& CI,
                     SkeletonASTVisitor& visitor) :
   SSTStringMapPragmaHandler("ImplicitState", plist, CI, visitor)
  {}
 private:
  SSTPragma* allocatePragma(const std::map<std::string, std::list<std::string>>& args) const override;
};


#endif
