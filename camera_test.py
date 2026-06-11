import time

def take_photo(filename):
    print("Снимаю фото:", filename)
    time.sleep(2)
    print("Готово")

def main():
    x = 5
    take_photo("photo1.jpg")
    take_photo("photo2.jpg")

main()