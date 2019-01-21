# test
# Loop similar to arduino setup loop!
def setup():
    print('Im in setup')
    pass

def loop():
    print('Im in loop')
    #return True 

status = True
setup()
while (not loop()):
    pass


