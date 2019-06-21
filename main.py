from PurePursuit import *

if __name__ == '__main__':
    cx = np.arange(0, 50, 1)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
    path = [cx, cy]
    follow_path(path)
