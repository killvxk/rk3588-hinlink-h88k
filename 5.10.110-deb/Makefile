

CC = gcc
CFLAGS = -c -O2 -g

MAIN_DIR = .
TARGET_BIN_DIR = $(MAIN_DIR)
TARGET_OBJ_DIR = $(MAIN_DIR)

BIN_NAME  = diaggrabpro

DIAGGRABPRO_DIR = $(MAIN_DIR)

SRC_DIR = $(DIAGGRABPRO_DIR)

INCLUDE_PREFIX = -I

ALL_PATHS= $(SRC_DIR)

ALL_INCLUDES = $(addprefix $(INCLUDE_PREFIX), $(ALL_PATHS))

OBJ_CMD = -o

LD_CMD = -o

LD_LIBRARY = -lpthread
LD_LIBLRT = -lrt

DIAGGRABPRO_OBJS =$(TARGET_OBJ_DIR)/diaggrabpro.o $(TARGET_OBJ_DIR)/ftp_client.o  $(TARGET_OBJ_DIR)/mdm.o

			   
BIN_OBJS = $(DIAGGRABPRO_OBJS)

$(TARGET_OBJ_DIR)/%.o:$(DIAGGRABPRO_DIR)/%.c
	@echo ---------------------------------------------------------
	@echo Build OBJECT $(@) from SOURCE $<
	@$(CC) $(CFLAGS) $(ALL_INCLUDES) $(OBJ_CMD) $@ $<
	@echo ---------------------------------------------------------

.PHONY: all clean

all:bin

bin:$(BIN_OBJS)
	@echo ---------------------------------------------------------
	@echo Create bin file $(BIN_NAME)
	@$(CC) $(LD_CMD) $(TARGET_BIN_DIR)/$(BIN_NAME) $^ $(LD_LIBRARY) $(LD_LIBLRT)
	@echo ---------------------------------------------------------
	
clean:
	@rm -fr $(BIN_OBJS) $(BIN_NAME)
	
