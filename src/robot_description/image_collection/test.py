from ultralytics import YOLO
model = YOLO("best.pt")
results = model("data_collection/red_box/red_box_000.jpg")
results[0].show()