from ultralytics import YOLO

def execute_hybrid_training():
    # Initialize the medium-sized YOLOv8 model (m-variant).
    # Rationale: A larger parameter capacity is strictly required here to prevent 
    # feature interference when learning contradictory domains (e.g. faces vs road blocks).
    model = YOLO('yolov8m.pt') 

    # Execute the training pipeline with constrained hardware parameters.
    model.train(
        data='data.yaml',         
        
        # Extended iterations allow the network to resolve complex gradient conflicts.
        epochs=75,                
        
        imgsz=640,                
        
        # Force computation on the primary dedicated GPU.
        device='0',               
        
        # Constrained batch size to compensate for the larger model and prevent VRAM overflow.
        batch=4,                  
        
        workers=2,                
        project='unified_vision', 
        name='multi_domain_model',
        
        # Mandate the generation of loss and precision graphs for post-training analysis.
        plots=True                
    )

if __name__ == '__main__':
    execute_hybrid_training()