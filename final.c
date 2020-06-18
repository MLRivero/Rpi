#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/miscdevice.h> // misc dev
#include <linux/fs.h>         // file operations
#include <asm/uaccess.h>      // copy to/from user space
#include <linux/wait.h>       // waiting queue
#include <linux/sched.h>      // TASK_INTERRUMPIBLE
#include <linux/delay.h>      // udelay
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/semaphore.h>  // librería

#define DRIVER_AUTHOR "Grupo Morado"
#define DRIVER_DESC   "Driver para placa"

//GPIOS numbers as in BCM RPi

#define GPIO_BUTTON1 2
#define GPIO_BUTTON2 3

#define GPIO_SPEAKER 4

#define GPIO_GREEN1  27
#define GPIO_GREEN2  22
#define GPIO_YELLOW1 17
#define GPIO_YELLOW2 11
#define GPIO_RED1    10
#define GPIO_RED2    9

static int LED_GPIOS[]= {GPIO_GREEN1, GPIO_GREEN2, GPIO_YELLOW1, GPIO_YELLOW2, GPIO_RED1, GPIO_RED2} ;

static char *led_desc[]= {"GPIO_GREEN1","GPIO_GREEN2","GPIO_YELLOW1","GPIO_YELLOW2","GPIO_RED1","GPIO_RED2"} ;

DECLARE_WAIT_QUEUE_HEAD(cola);
DEFINE_SEMAPHORE(semaforo);  // declaración de un semáforo abierto (valor=1)

static char buffer[PAGE_SIZE];
static int w_ptr = 0;
static int r_ptr = 0;
static short hayDato = 0;

static int time = 200; // tiempo en ms
module_param(time, int, S_IRUGO);

static struct timer_list timeHandler1;
static struct timer_list timeHandler2;

static void f_timeHandler(unsigned long data);

static void f_bottomHalf1(unsigned long);
static void f_bottomHalf2(unsigned long);

DECLARE_TASKLET(bottomHalf1, f_bottomHalf1, 0);
DECLARE_TASKLET(bottomHalf2, f_bottomHalf2, 0);

// declarar dos timers

/****************************************************************************/
/* Interrupts variables block                                               */
/****************************************************************************/
static short int irq_BUTTON1		= 0;
static short int irq_BUTTON2		= 1;
 
// text below will be seen in 'cat /proc/interrupt' command
#define GPIO_BUTTON1_DESC           "Boton 1"
#define GPIO_BUTTON2_DESC           "Boton 2"
 
// below is optional, used in more complex code, in our case, this could be
#define GPIO_BUTTON1_DEVICE_DESC	"Nove compadre"
#define GPIO_BUTTON2_DEVICE_DESC	"Eoo"

static void f_timeHandler(unsigned long data){
	if(data) {
		//printk(KERN_NOTICE "Enable interrupt 1");
		enable_irq(irq_BUTTON1);  
	} else {
		//printk(KERN_NOTICE "Enable interrupt 2"); 
		enable_irq(irq_BUTTON2);
		}
}


static void byte2leds(char ch)
{

	    int i;
    int j;
    int val=(int)ch;
    
    // Guardo los 2 primero de la OP
	for(j = 6; j <8;j++){
		printk("valor val a guardar = %i\n",((val >> j) & 1));
			LED_GPIOS[j] = (val >> j) & 1;
	}
	
	//((val >> i) & 1) es cada digito
	
	// Valores para los LEDS
	if(LED_GPIOS[6] == 0 && LED_GPIOS[7] == 0){
		for(i=0; i < 6; i++){
		printk("valor val = %i\n",((val >> j) & 1));
		 gpio_set_value(LED_GPIOS[i], (val >> i) & 1);
	 }
	}else if(LED_GPIOS[6] == 1 && LED_GPIOS[7] == 0){
		for(i=0; i < 6; i++){
		printk("valor val = %i\n",((val >> j) & 1));
		if(((val >> i) & 1) == 1){
			gpio_set_value(LED_GPIOS[i], (val >> i) | 0);
		}		 
	 }
	}else if(LED_GPIOS[6] == 0 && LED_GPIOS[7] == 1){
		for(i=0; i < 6; i++){
		printk("valor val = %i\n",((val >> j) & 1));
			if(((val >> i) & 1) == 1){
				gpio_set_value(LED_GPIOS[i], (val >> i) & 0);
			}
		}
	}
}

static char leds2byte(void)
{
    int val;
    char ch;
    int i;
    ch=0;

    for(i=0; i<6; i++)
    {
        val=gpio_get_value(LED_GPIOS[i]);
        ch= ch | (val << i);
    }
    return ch;
}

/****************************************************************************/
/* LEDs device file operations                                              */
/****************************************************************************/

static ssize_t leds_write(struct file *file, const char __user *buf,
                          size_t count, loff_t *ppos)
{

    char ch;

    if (copy_from_user( &ch, buf, 1 )) {
        return -EFAULT;
    }

    printk( KERN_INFO " (write) valor recibido: %d\n",(int)ch);

    byte2leds(ch);

    return 1;
}

static ssize_t leds_read(struct file *file, char __user *buf,
                         size_t count, loff_t *ppos)
{
    char ch;

    if(*ppos==0) *ppos+=1;
    else return 0;

    ch=leds2byte();

    printk( KERN_INFO " (read) valor entregado: %d\n",(int)ch);


    if(copy_to_user(buf,&ch,1)) return -EFAULT;

    return 1;
}

static const struct file_operations leds_fops = {
    .owner	= THIS_MODULE,
    .write	= leds_write,
    .read	= leds_read,
};

/****************************************************************************/
/* LEDs device struct                                                       */

static struct miscdevice leds_miscdev = {
    .minor	= MISC_DYNAMIC_MINOR,
    .name	= "leds",
    .fops	= &leds_fops,
};


/****************************************************************************/
/* IRQ handler - fired on interrupt                                         */
/****************************************************************************/



static irqreturn_t r_irq_handler1(int irq, void *dev_id, struct pt_regs *regs) {
 
    //int ch;
    // we will increment value in leds with button push
    // due to switch bouncing this hadler will be fired few times for every putton push
	//printk(KERN_NOTICE "Disable interrupt 1");
    disable_irq_nosync(irq_BUTTON1);
    mod_timer(&timeHandler1, jiffies + HZ * time / 1000 );
    
    tasklet_schedule(&bottomHalf1);
 
    return IRQ_HANDLED;
}
 
 
// manejador de la interrupcion dos
static irqreturn_t r_irq_handler2(int irq, void *dev_id, struct pt_regs *regs) {
 
	// int ch;
    // we will decrement value in leds with button push
    // due to switch bouncing this hadler will be fired few times for every putton push
   
	//printk(KERN_NOTICE "Disable interrupt 2");
    disable_irq_nosync(irq_BUTTON2);
    mod_timer(&timeHandler2, jiffies + HZ * time / 1000 );
   
	tasklet_schedule(&bottomHalf2);
   
 
    return IRQ_HANDLED;
}
 
 static void f_bottomHalf1(unsigned long unused){
	if (down_interruptible(&semaforo)) //return -ERESTARTSYS; // capturamos el semáforo 
	buffer[w_ptr] = 'a';
	buffer[w_ptr] = '1';
	w_ptr += 1;
	hayDato = 1;
    printk(KERN_NOTICE "(WRITE) Se ha escrito un %c, actualmente el string es %s, y tiene %d caracteres", buffer[w_ptr-1], buffer, strlen(buffer) );
	up(&semaforo);
	wake_up_interruptible(&cola);
}

static void f_bottomHalf2(unsigned long unused) {
	if (down_interruptible(&semaforo)) //return -ERESTARTSYS; // capturamos el semáforo
	buffer[w_ptr] = 'b';
	buffer[w_ptr] = '2';
	w_ptr += 1;
    hayDato = 1;
    printk(KERN_NOTICE "(WRITE) Se ha escrito un %c, actualmente el string es %s, y tiene %d caracteres", buffer[w_ptr-1], buffer, strlen(buffer) );
    up(&semaforo);
    wake_up_interruptible(&cola);
}

static ssize_t speaker_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{

    char ch;

    if (copy_from_user( &ch, buf, 1 )) {
        return -EFAULT;
    }
    
    if(ch=='0') {
    gpio_set_value(4, 0);
	//printk( KERN_INFO " (write) valor recibido: 0\n");
	} else if (ch=='1') {
    gpio_set_value(4,1);
    //printk( KERN_INFO " (write) valor recibido: 1\n");
	} else {
		return 1;
	}

    return 1;
}

static const struct file_operations speaker_fops = {
    .owner	= THIS_MODULE,
    .write	= speaker_write,
//    .read	= speaker_read,
};

/****************************************************************************/
/* LEDs device struct                                                       */

static struct miscdevice speaker_miscdev = {
    .minor	= MISC_DYNAMIC_MINOR,
    .name	= "speaker",
    .fops	= &speaker_fops,
};


/****************************************************************************/
/* Buttons device file operations                                              */
/****************************************************************************/

static ssize_t buttons_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	/* Coger el semáforo, comprobar si se han introducido datos. Si no, se suelta y se queda esperando la cola.
	 * Cuando sale de la cola, adquiere el semáforo y sale del bucle.
	 * Una vez hecho esto, calcula la cantidad de bytes que va a devolver.
	 * Devuelve los bytes con copy_to_user y da la cantidad de bytes devueltos como valor de return.
	 * Suelta el semáforo y hasta luego.
	 * 
	 */
	//char out[w_ptr - r_ptr - 1];
	int size = 0;
	int i = r_ptr;
	int j = 0;
	ssize_t retval = 0;
	// hay que proteger el acceso a thereIsData
	if (down_interruptible(&semaforo)) return -ERESTARTSYS; // capturamos el semáforo
	while (!hayDato){
		up(&semaforo);
		if(wait_event_interruptible(cola, hayDato))return -ERESTARTSYS;
		if (down_interruptible(&semaforo)) return -ERESTARTSYS; // capturamos el semáforo
	}
	if ( (*ppos >= PAGE_SIZE) | (r_ptr >= PAGE_SIZE) ){
		printk(KERN_NOTICE "Hemos llegado al final del buffer");
		up (&semaforo);
        return retval;
	}
	//if (r_ptr == 0){ size = w_ptr - r_ptr } else {
	//size = w_ptr - r_ptr - 1; }
	size = w_ptr - r_ptr;
	char out[size];
	hayDato = 0;
	//if (r_ptr == 0) { i = r_ptr } else {
	//i = r_ptr + 1;}
	
	printk("(read) Se mostrarán %d bytes, dsede el %d hasta el %d", size, r_ptr, w_ptr);
	
	//for (i; buffer[i] != '\0'; i++){
	for (i; i < w_ptr; i++){
		out[j] = buffer[i];
		j++;
	}
	r_ptr = i;
	printk("(read) hasta la posicion %d", i - 1);
    
	count = (count > size)? size : count;
	
	if (copy_to_user(buf, out, count)){
		return -EFAULT;
	}
	*ppos += size;
	retval = count;
	printk(KERN_NOTICE "(read) Se han devuelto %d pulsaciones, siguiente byte a leer %d", (int) retval, r_ptr);
	up (&semaforo);
	return retval;
	
    //i = mostrados;
    //for (i; buffer[i] != '\0'; i++)
    //size = i;
    //hayDato = count<size;
    //count=(count>size)? size : count; // si contador mayor o igual pone size, si no, pone count
    //if(copy_to_user(buf, buffer, count)){
		//return -EFAULT;
	//}
	
	//for (i = mostrados; mostrados < introducidos; mostrados++){
		//sprintf("%d", nuevo);
		//count++;
	//}
	
	//if(copy_to_user(buf, nuevo, count) ){
		//return -EFAULT;
	//}
	
	// cuando hago un read se borra lo demas
}

static const struct file_operations buttons_fops = {
    .owner	= THIS_MODULE,
    //.write	= buttons_write,
    .read	= buttons_read,
};

/****************************************************************************/
/* Buttons device struct                                                       */
/****************************************************************************/

static struct miscdevice buttons_miscdev = {
    .minor	= MISC_DYNAMIC_MINOR,
    .name	= "buttons",
    .fops	= &buttons_fops,
};

/*****************************************************************************/
/* This functions registers devices, requests GPIOs and configures interrupts */
/*****************************************************************************/

/*******************************
 *  register device for leds
 *******************************/

static int r_dev_config(void)
{
    int ret=0;
    ret = misc_register(&buttons_miscdev);
    if (ret < 0) {
        printk(KERN_ERR "misc_register failed\n");
    }
	else
		printk(KERN_NOTICE "misc_register OK... buttons_miscdev.minor=%d\n", buttons_miscdev.minor);
	return ret;
}

/*******************************
 *  set interrup for buttons
 *******************************/
 
static int r_int_config(void)
{
    int res=0;
    int res1=0;
    if ((res=gpio_request(GPIO_BUTTON1, GPIO_BUTTON1_DESC))) {
        printk(KERN_ERR "GPIO request faiure: %s, error %d\n", GPIO_BUTTON1_DESC, res);
        return res;
    }
    /*gpio_request boton 2*/
    if ((res1=gpio_request(GPIO_BUTTON2, GPIO_BUTTON2_DESC))) {
        printk(KERN_ERR "GPIO request faiure: %s, error %d\n", GPIO_BUTTON2_DESC, res1);
        return res1;
    }
   
 
    if ( (irq_BUTTON1 = gpio_to_irq(GPIO_BUTTON1)) < 0 ) {
        printk(KERN_ERR "GPIO to IRQ mapping faiure %s, error code %d\n", GPIO_BUTTON1_DESC, irq_BUTTON1);
        return irq_BUTTON1;
    }
   
    if ( (irq_BUTTON2 = gpio_to_irq(GPIO_BUTTON2)) < 0 ) {
        printk(KERN_ERR "GPIO to IRQ mapping faiure %s, error code %d\n", GPIO_BUTTON2_DESC, irq_BUTTON2);
        return irq_BUTTON2;
    }
 
    printk(KERN_NOTICE "  Mapped int %d for button1 in gpio %d\n", irq_BUTTON1, GPIO_BUTTON1);
    printk(KERN_NOTICE "  Mapped int %d for button2 in gpio %d\n", irq_BUTTON2, GPIO_BUTTON2);
 
    if ((res=request_irq(irq_BUTTON1,
                    (irq_handler_t ) r_irq_handler1,
                    IRQF_TRIGGER_FALLING,
                    GPIO_BUTTON1_DESC,
                    GPIO_BUTTON1_DEVICE_DESC))) {
        printk(KERN_ERR "Irq Request failure, error %d\n", res);
        return res;
    }
   
     if ((res1=request_irq(irq_BUTTON2,
                    (irq_handler_t ) r_irq_handler2,
                    IRQF_TRIGGER_FALLING,
                    GPIO_BUTTON2_DESC,
                    GPIO_BUTTON2_DEVICE_DESC))) {
        printk(KERN_ERR "Irq Request failure, error %d\n", res1);
        return res1;
    } 
 
    return res && res1;
}

static int r_GPIO_config(void)
{
    int i;
    int res=0;
    for(i=0; i<6; i++)
    {
        if ((res=gpio_request_one(LED_GPIOS[i], GPIOF_INIT_LOW, led_desc[i]))) 
        {
            printk(KERN_ERR "GPIO request faiure: led GPIO %d %s\n",LED_GPIOS[i], led_desc[i]);
            return res;
        }
        gpio_direction_output(LED_GPIOS[i],0);
	}
	return res;
}

static int r_dev_config_speaker(void)
{
    int ret=0;
    ret = misc_register(&speaker_miscdev);
    if (ret < 0) {
        printk(KERN_ERR "misc_register failed\n");
    }
	else
		printk(KERN_NOTICE "misc_register OK... speaker_miscdev.minor=%d\n", speaker_miscdev.minor);
	gpio_direction_output(4,0);
	return ret;
}

static int r_dev_config_led(void)
{
    int ret=0;
    ret = misc_register(&leds_miscdev);
    if (ret < 0) {
        printk(KERN_ERR "misc_register failed\n");
    }
	else
		printk(KERN_NOTICE "misc_register OK... leds_miscdev.minor=%d\n", leds_miscdev.minor);
	return ret;
}

/****************************************************************************/
/* Module init / cleanup block.                                             */
/****************************************************************************/

static void r_cleanup(void) {
    int ret;
    int i;
    ret = del_timer(&timeHandler1);
    if (ret) { printk(KERN_NOTICE "Timer still in use\n");}
    for(i=0; i<6; i++){
    gpio_free(LED_GPIOS[i]);}
    printk(KERN_NOTICE "Uninstalled timer module\n");
    printk(KERN_NOTICE "%s module cleaning up...\n", KBUILD_MODNAME);
    if(irq_BUTTON1) free_irq(irq_BUTTON1, GPIO_BUTTON1_DEVICE_DESC);   //libera irq
    gpio_free(GPIO_BUTTON1);  // libera GPIO
    if(irq_BUTTON2) free_irq(irq_BUTTON2, GPIO_BUTTON2_DEVICE_DESC);   //libera irq
    gpio_free(GPIO_BUTTON2);  // libera GPIO
    if (buttons_miscdev.this_device) misc_deregister(&buttons_miscdev);
    if (speaker_miscdev.this_device) misc_deregister(&speaker_miscdev);
    if (leds_miscdev.this_device) misc_deregister(&leds_miscdev);
    printk(KERN_NOTICE "Done. Bye from %s module\n", KBUILD_MODNAME);
    return;
}

static int r_init(void) {
    int res=0;
    printk(KERN_NOTICE "Hello, loading %s module!\n", KBUILD_MODNAME);
   
    printk(KERN_NOTICE "%s - int config...\n", KBUILD_MODNAME);
    if(( res = r_int_config() ))
    {
        r_cleanup();
        return res;
    }
    
    if (( res = r_dev_config() )){
		r_cleanup();
		return res;
	}
	
	if (( res = r_dev_config_speaker() )){
		r_cleanup();
		return res;
	}
	
	if (( res = r_dev_config_led() )){
		r_cleanup();
		return res;
	}
	
	if((res = r_GPIO_config()))
    {
		r_cleanup();
		return res;
	}
	
	setup_timer(&timeHandler1, f_timeHandler, 1);
	setup_timer(&timeHandler2, f_timeHandler, 0);

    return res;
}

module_init(r_init);
module_exit(r_cleanup);

/****************************************************************************/
/* Module licensing/description block.                                      */
/****************************************************************************/
MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
