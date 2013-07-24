# GNU Make project makefile autogenerated by Premake
ifndef config
  config=debug32
endif

ifndef verbose
  SILENT = @
endif

ifndef CC
  CC = gcc
endif

ifndef CXX
  CXX = g++
endif

ifndef AR
  AR = ar
endif

ifndef RESCOMP
  ifdef WINDRES
    RESCOMP = $(WINDRES)
  else
    RESCOMP = windres
  endif
endif

ifeq ($(config),debug32)
  OBJDIR     = obj/x32/Debug/evds_tests
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/evds_testsd32
  DEFINES   += -DDEBUG
  INCLUDES  += -I../../include -I../../external/simc/include -I../../tests
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -m32
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m32 -L/usr/lib32 -lstdc++
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/libevdsd32.a ../../bin/libsimcd32.a -lm -lpthread
  LDDEPS    += ../../bin/libevdsd32.a ../../bin/libsimcd32.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),release32)
  OBJDIR     = obj/x32/Release/evds_tests
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/evds_tests32
  DEFINES   += 
  INCLUDES  += -I../../include -I../../external/simc/include -I../../tests
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -g -m32
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m32 -L/usr/lib32 -lstdc++
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/libevds32.a ../../bin/libsimc32.a -lm -lpthread
  LDDEPS    += ../../bin/libevds32.a ../../bin/libsimc32.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),debugdynamic32)
  OBJDIR     = obj/x32/DebugDynamic/evds_tests
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/evds_testsd32
  DEFINES   += -DDEBUG -DEVDS_DYNAMIC -DIVSS_DYNAMIC -DRDRS_DYNAMIC -DSIMC_DYNAMIC
  INCLUDES  += -I../../include -I../../external/simc/include -I../../tests
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -m32
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m32 -L/usr/lib32 -lstdc++
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += -levdsd32 ../../bin/libsimcd32.a -lm -lpthread
  LDDEPS    += ../../bin/libevdsd32.so ../../bin/libsimcd32.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),releasedynamic32)
  OBJDIR     = obj/x32/ReleaseDynamic/evds_tests
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/evds_tests32
  DEFINES   += -DEVDS_DYNAMIC -DIVSS_DYNAMIC -DRDRS_DYNAMIC -DSIMC_DYNAMIC
  INCLUDES  += -I../../include -I../../external/simc/include -I../../tests
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -g -m32
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m32 -L/usr/lib32 -lstdc++
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += -levds32 ../../bin/libsimc32.a -lm -lpthread
  LDDEPS    += ../../bin/libevds32.so ../../bin/libsimc32.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),debugsinglethread32)
  OBJDIR     = obj/x32/DebugSingleThread/evds_tests
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/evds_tests_std32
  DEFINES   += -DDEBUG -DEVDS_SINGLETHREADED -DIVSS_SINGLETHREADED -DRDRS_SINGLETHREADED -DSIMC_SINGLETHREADED
  INCLUDES  += -I../../include -I../../external/simc/include -I../../tests
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -m32
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m32 -L/usr/lib32 -lstdc++
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/libevds_std32.a ../../bin/libsimc_std32.a -lm -lpthread
  LDDEPS    += ../../bin/libevds_std32.a ../../bin/libsimc_std32.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),releasesinglethread32)
  OBJDIR     = obj/x32/ReleaseSingleThread/evds_tests
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/evds_tests_st32
  DEFINES   += -DEVDS_SINGLETHREADED -DIVSS_SINGLETHREADED -DRDRS_SINGLETHREADED -DSIMC_SINGLETHREADED
  INCLUDES  += -I../../include -I../../external/simc/include -I../../tests
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -g -m32
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m32 -L/usr/lib32 -lstdc++
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/libevds_st32.a ../../bin/libsimc_st32.a -lm -lpthread
  LDDEPS    += ../../bin/libevds_st32.a ../../bin/libsimc_st32.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),debugsinglethreaddynamic32)
  OBJDIR     = obj/x32/DebugSingleThreadDynamic/evds_tests
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/evds_tests_std32
  DEFINES   += -DDEBUG -DEVDS_DYNAMIC -DIVSS_DYNAMIC -DRDRS_DYNAMIC -DSIMC_DYNAMIC -DEVDS_SINGLETHREADED -DIVSS_SINGLETHREADED -DRDRS_SINGLETHREADED -DSIMC_SINGLETHREADED
  INCLUDES  += -I../../include -I../../external/simc/include -I../../tests
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -m32
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m32 -L/usr/lib32 -lstdc++
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += -levds_std32 ../../bin/libsimc_std32.a -lm -lpthread
  LDDEPS    += ../../bin/libevds_std32.so ../../bin/libsimc_std32.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),releasesinglethreaddynamic32)
  OBJDIR     = obj/x32/ReleaseSingleThreadDynamic/evds_tests
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/evds_tests_st32
  DEFINES   += -DEVDS_DYNAMIC -DIVSS_DYNAMIC -DRDRS_DYNAMIC -DSIMC_DYNAMIC -DEVDS_SINGLETHREADED -DIVSS_SINGLETHREADED -DRDRS_SINGLETHREADED -DSIMC_SINGLETHREADED
  INCLUDES  += -I../../include -I../../external/simc/include -I../../tests
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -g -m32
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m32 -L/usr/lib32 -lstdc++
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += -levds_st32 ../../bin/libsimc_st32.a -lm -lpthread
  LDDEPS    += ../../bin/libevds_st32.so ../../bin/libsimc_st32.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),debug64)
  OBJDIR     = obj/x64/Debug/evds_tests
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/evds_testsd
  DEFINES   += -DDEBUG
  INCLUDES  += -I../../include -I../../external/simc/include -I../../tests
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m64 -L/usr/lib64 -lstdc++
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/libevdsd.a ../../bin/libsimcd.a -lm -lpthread
  LDDEPS    += ../../bin/libevdsd.a ../../bin/libsimcd.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),release64)
  OBJDIR     = obj/x64/Release/evds_tests
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/evds_tests
  DEFINES   += 
  INCLUDES  += -I../../include -I../../external/simc/include -I../../tests
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -g -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m64 -L/usr/lib64 -lstdc++
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/libevds.a ../../bin/libsimc.a -lm -lpthread
  LDDEPS    += ../../bin/libevds.a ../../bin/libsimc.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),debugdynamic64)
  OBJDIR     = obj/x64/DebugDynamic/evds_tests
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/evds_testsd
  DEFINES   += -DDEBUG -DEVDS_DYNAMIC -DIVSS_DYNAMIC -DRDRS_DYNAMIC -DSIMC_DYNAMIC
  INCLUDES  += -I../../include -I../../external/simc/include -I../../tests
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m64 -L/usr/lib64 -lstdc++
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += -levdsd ../../bin/libsimcd.a -lm -lpthread
  LDDEPS    += ../../bin/libevdsd.so ../../bin/libsimcd.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),releasedynamic64)
  OBJDIR     = obj/x64/ReleaseDynamic/evds_tests
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/evds_tests
  DEFINES   += -DEVDS_DYNAMIC -DIVSS_DYNAMIC -DRDRS_DYNAMIC -DSIMC_DYNAMIC
  INCLUDES  += -I../../include -I../../external/simc/include -I../../tests
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -g -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m64 -L/usr/lib64 -lstdc++
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += -levds ../../bin/libsimc.a -lm -lpthread
  LDDEPS    += ../../bin/libevds.so ../../bin/libsimc.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),debugsinglethread64)
  OBJDIR     = obj/x64/DebugSingleThread/evds_tests
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/evds_tests_std
  DEFINES   += -DDEBUG -DEVDS_SINGLETHREADED -DIVSS_SINGLETHREADED -DRDRS_SINGLETHREADED -DSIMC_SINGLETHREADED
  INCLUDES  += -I../../include -I../../external/simc/include -I../../tests
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m64 -L/usr/lib64 -lstdc++
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/libevds_std.a ../../bin/libsimc_std.a -lm -lpthread
  LDDEPS    += ../../bin/libevds_std.a ../../bin/libsimc_std.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),releasesinglethread64)
  OBJDIR     = obj/x64/ReleaseSingleThread/evds_tests
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/evds_tests_st
  DEFINES   += -DEVDS_SINGLETHREADED -DIVSS_SINGLETHREADED -DRDRS_SINGLETHREADED -DSIMC_SINGLETHREADED
  INCLUDES  += -I../../include -I../../external/simc/include -I../../tests
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -g -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m64 -L/usr/lib64 -lstdc++
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/libevds_st.a ../../bin/libsimc_st.a -lm -lpthread
  LDDEPS    += ../../bin/libevds_st.a ../../bin/libsimc_st.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),debugsinglethreaddynamic64)
  OBJDIR     = obj/x64/DebugSingleThreadDynamic/evds_tests
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/evds_tests_std
  DEFINES   += -DDEBUG -DEVDS_DYNAMIC -DIVSS_DYNAMIC -DRDRS_DYNAMIC -DSIMC_DYNAMIC -DEVDS_SINGLETHREADED -DIVSS_SINGLETHREADED -DRDRS_SINGLETHREADED -DSIMC_SINGLETHREADED
  INCLUDES  += -I../../include -I../../external/simc/include -I../../tests
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m64 -L/usr/lib64 -lstdc++
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += -levds_std ../../bin/libsimc_std.a -lm -lpthread
  LDDEPS    += ../../bin/libevds_std.so ../../bin/libsimc_std.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),releasesinglethreaddynamic64)
  OBJDIR     = obj/x64/ReleaseSingleThreadDynamic/evds_tests
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/evds_tests_st
  DEFINES   += -DEVDS_DYNAMIC -DIVSS_DYNAMIC -DRDRS_DYNAMIC -DSIMC_DYNAMIC -DEVDS_SINGLETHREADED -DIVSS_SINGLETHREADED -DRDRS_SINGLETHREADED -DSIMC_SINGLETHREADED
  INCLUDES  += -I../../include -I../../external/simc/include -I../../tests
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -g -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m64 -L/usr/lib64 -lstdc++
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += -levds_st ../../bin/libsimc_st.a -lm -lpthread
  LDDEPS    += ../../bin/libevds_st.so ../../bin/libsimc_st.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

OBJECTS := \
	$(OBJDIR)/framework.o \
	$(OBJDIR)/tests_evds_system.o \

RESOURCES := \

SHELLTYPE := msdos
ifeq (,$(ComSpec)$(COMSPEC))
  SHELLTYPE := posix
endif
ifeq (/bin,$(findstring /bin,$(SHELL)))
  SHELLTYPE := posix
endif

.PHONY: clean prebuild prelink

all: $(TARGETDIR) $(OBJDIR) prebuild prelink $(TARGET)
	@:

$(TARGET): $(GCH) $(OBJECTS) $(LDDEPS) $(RESOURCES)
	@echo Linking evds_tests
	$(SILENT) $(LINKCMD)
	$(POSTBUILDCMDS)

$(TARGETDIR):
	@echo Creating $(TARGETDIR)
ifeq (posix,$(SHELLTYPE))
	$(SILENT) mkdir -p $(TARGETDIR)
else
	$(SILENT) mkdir $(subst /,\\,$(TARGETDIR))
endif

$(OBJDIR):
	@echo Creating $(OBJDIR)
ifeq (posix,$(SHELLTYPE))
	$(SILENT) mkdir -p $(OBJDIR)
else
	$(SILENT) mkdir $(subst /,\\,$(OBJDIR))
endif

clean:
	@echo Cleaning evds_tests
ifeq (posix,$(SHELLTYPE))
	$(SILENT) rm -f  $(TARGET)
	$(SILENT) rm -rf $(OBJDIR)
else
	$(SILENT) if exist $(subst /,\\,$(TARGET)) del $(subst /,\\,$(TARGET))
	$(SILENT) if exist $(subst /,\\,$(OBJDIR)) rmdir /s /q $(subst /,\\,$(OBJDIR))
endif

prebuild:
	$(PREBUILDCMDS)

prelink:
	$(PRELINKCMDS)

ifneq (,$(PCH))
$(GCH): $(PCH)
	@echo $(notdir $<)
ifeq (posix,$(SHELLTYPE))
	-$(SILENT) cp $< $(OBJDIR)
else
	$(SILENT) xcopy /D /Y /Q "$(subst /,\,$<)" "$(subst /,\,$(OBJDIR))" 1>nul
endif
	$(SILENT) $(CC) $(CFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
endif

$(OBJDIR)/framework.o: ../../tests/framework.c
	@echo $(notdir $<)
	$(SILENT) $(CC) $(CFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/tests_evds_system.o: ../../tests/tests_evds_system.c
	@echo $(notdir $<)
	$(SILENT) $(CC) $(CFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"

-include $(OBJECTS:%.o=%.d)
