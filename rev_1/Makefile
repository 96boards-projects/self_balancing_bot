CC := gcc
LDFLAGS := -lmraa -lm
CFLAGS := -Wall -Wextra -g3
SRCDIR := src
INCDIR := inc
OBJDIR := obj

all: dirs motor_control

motor_control: $(OBJDIR)/motor_control.o $(OBJDIR)/pid.o
	@$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(OBJDIR)/%.o: $(SRCDIR)/%.c
	@$(CC) $(CFLAGS) -I./$(INCDIR) -o $@ -c $^

dirs:
	@mkdir -p $(OBJDIR)

clean:
	@rm -rf $(OBJDIR) motor_control

.PHONY: all dirs clean
