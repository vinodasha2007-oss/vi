# generate_urdf.py
# Этот файл не используется, так как rocket.urdf уже создан вручную
# Он оставлен для возможного будущего использования

import os

print("=" * 50)
print("ГЕНЕРАТОР URDF (не используется)")
print("=" * 50)
print("rocket.urdf уже создан вручную и ссылается на rocket.obj")
print("Для работы просто поместите rocket.obj в ту же папку")

# Проверка наличия rocket.obj
if os.path.exists("rocket.obj"):
    print("\n✅ rocket.obj найден! Всё готово к запуску.")
    print("   Запустите: python main.py")
else:
    print("\n❌ rocket.obj не найден!")
    print("   Поместите ваш 3D файл rocket.obj в текущую папку")

print("=" * 50)
