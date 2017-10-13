CC := g++
LDFLAGS := -lmraa -lm
CFLAGS := -Wall -Wextra -g3
SRCDIR := src
INCDIR := inc
OBJDIR := obj

all: dirs motor_control

motor_control: $(OBJDIR)/motor_control.o $(OBJDIR)/pid.o $(OBJDIR)/I2Cdev.o $(OBJDIR)/MPU6050.o
	@$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(OBJDIR)/%.o: $(SRCDIR)/%.c
	@$(CC) $(CFLAGS) -I./$(INCDIR) -o $@ -c $^

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	@$(CC) $(CFLAGS) -I./$(INCDIR) -o $@ -c $^

dirs:
	@mkdir -p $(OBJDIR)

clean:
	@rm -rf $(OBJDIR) motor_control

.PHONY: all dirs clean
