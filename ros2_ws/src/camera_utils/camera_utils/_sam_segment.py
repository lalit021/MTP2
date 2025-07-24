import numpy as np
import torch
import matplotlib.pyplot as plt
from PIL import Image as PILImage
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
import cv2


class SAM2MaskPredictor:
    def __init__(self, model_cfg, checkpoint_path, device=None):
        """
        Initializes the SAM2 Mask Predictor with a configuration and checkpoint.
        """
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.sam2_model = build_sam2(model_cfg, checkpoint_path, device=self.device)
        self.predictor = SAM2ImagePredictor(self.sam2_model)

    def set_image(self, image_input):
        """
        Accepts an image object (NumPy array or PIL Image).
        Converts the input into a suitable format for the segmentation model.
        """
        # Check if the input is a NumPy array or a PIL image
        if isinstance(image_input, np.ndarray):
            # If it's a NumPy array, assume it's already an image
            image = PILImage.fromarray(image_input)
        elif isinstance(image_input, PILImage.Image):
            # If it's already a PIL image, just use it
            image = image_input
        else:
            raise TypeError("Input must be a NumPy array or PIL Image.")

        # Convert to RGB if it's not already
        image = np.array(image.convert("RGB"))
        self.predictor.set_image(image)

    def predict_with_points(self, point_coords, point_labels, multimask_output=True):
        """
        Predicts the masks based on points provided (foreground or background points).
        """
        masks, scores, logits = self.predictor.predict(
            point_coords=point_coords,
            point_labels=point_labels,
            multimask_output=multimask_output,
        )
        sorted_ind = np.argsort(scores)[::-1]
        masks = masks[sorted_ind]
        scores = scores[sorted_ind]
        logits = logits[sorted_ind]
        return masks, scores, logits

    def predict_with_box(self, box_coords, multimask_output=True):
        """
        Predicts the masks based on the box coordinates provided.
        """
        masks, scores, logits = self.predictor.predict(
            point_coords=None,
            point_labels=None,
            box=box_coords,
            multimask_output=multimask_output,
        )
        return masks, scores, logits

    def predict_with_combined_prompts(self, point_coords, point_labels, box_coords, multimask_output=True):
        """
        Predicts the masks using both points and box as inputs.
        """
        masks, scores, logits = self.predictor.predict(
            point_coords=point_coords,
            point_labels=point_labels,
            box=box_coords,
            multimask_output=multimask_output,
        )
        sorted_ind = np.argsort(scores)[::-1]
        masks = masks[sorted_ind]
        scores = scores[sorted_ind]
        logits = logits[sorted_ind]
        return masks, scores, logits

    def display_results(self, image, masks, scores, point_coords=None, box_coords=None, input_labels=None, borders=True, display_mode="all"):
        """
        Display the image along with the predicted masks and other details like points or boxes.

        Args:
            image (PIL.Image): The input image.
            masks (list): List of masks to display.
            scores (list): Scores corresponding to each mask.
            point_coords (np.array, optional): Coordinates of points to display.
            box_coords (np.array, optional): Coordinates of boxes to display.
            input_labels (np.array, optional): Labels for the points.
            borders (bool, optional): Whether to display borders around masks. Defaults to True.
            display_mode (str, optional): Display mode - 'all' to show all results, 'top' to show the top result. Defaults to "all".
        """
        if display_mode == "top":
            # Display only the mask with the highest score
            top_idx = np.argmax(scores)
            plt.figure(figsize=(10, 10))
            plt.imshow(image)
            self.show_mask(masks[top_idx], plt.gca(), borders=borders)
            if point_coords is not None:
                assert input_labels is not None
                self.show_points(point_coords, input_labels, plt.gca())
            if box_coords is not None:
                self.show_box(box_coords, plt.gca())
            plt.title(f"Top Mask, Score: {scores[top_idx]:.3f}", fontsize=18)
            plt.axis('off')
            plt.show()
        elif display_mode == "all":
            # Display all masks
            for i, (mask, score) in enumerate(zip(masks, scores)):
                plt.figure(figsize=(10, 10))
                plt.imshow(image)
                self.show_mask(mask, plt.gca(), borders=borders)
                if point_coords is not None:
                    assert input_labels is not None
                    self.show_points(point_coords, input_labels, plt.gca())
                if box_coords is not None:
                    self.show_box(box_coords, plt.gca())
                plt.title(f"Mask {i+1}, Score: {score:.3f}", fontsize=18)
                plt.axis('off')
                plt.show()
        else:
            raise ValueError("Invalid display_mode. Choose 'all' or 'top'.")

    def show_mask(self, mask, ax, random_color=False, borders=True):
        """
        Visualize the mask on the image.
        """
        if random_color:
            color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
        else:
            color = np.array([30/255, 144/255, 255/255, 0.6])
        h, w = mask.shape[-2:]
        mask = mask.astype(np.uint8)
        mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
        if borders:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            contours = [cv2.approxPolyDP(contour, epsilon=0.01, closed=True) for contour in contours]
            mask_image = cv2.drawContours(mask_image, contours, -1, (1, 1, 1, 0.5), thickness=2)
        ax.imshow(mask_image)

    def show_points(self, coords, labels, ax, marker_size=375):
        """
        Display the points (foreground or background) on the image.
        """
        pos_points = coords[labels == 1]
        neg_points = coords[labels == 0]
        ax.scatter(pos_points[:, 0], pos_points[:, 1], color='green', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)
        ax.scatter(neg_points[:, 0], neg_points[:, 1], color='red', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)

    def show_box(self, box, ax):
        """
        Display a bounding box on the image.
        """
        x0, y0 = box[0], box[1]
        w, h = box[2] - box[0], box[3] - box[1]
        ax.add_patch(plt.Rectangle((x0, y0), w, h, edgecolor='green', facecolor=(0, 0, 0, 0), lw=2))


if __name__ == "__main__":
    
    # Example configuration and checkpoint paths (replace these with actual paths)
    model_cfg = "configs/sam2.1/sam2.1_hiera_t.yaml"  # Replace with your model config path
    checkpoint_path = "/home/friday/MTP2/sam2/checkpoints/sam2.1_hiera_tiny.pt"  # Replace with your model checkpoint path

    # Initialize the mask predictor
    predictor = SAM2MaskPredictor(model_cfg=model_cfg, checkpoint_path=checkpoint_path)

    # Load an example image
    image_path = "/home/friday/MTP2/sam2/notebooks/images/truck.jpg"  # Replace with the path to your test image
    image = PILImage.open(image_path)

    # Set the image for the predictor
    predictor.set_image(image)

    # Example: Predict masks based on some points (foreground and background)
    # Example point coordinates and labels (you would normally extract these based on user input or annotations)
    # Define point coordinates and labels for prediction
    input_point = np.array([[500, 375]])  # Example point coordinates
    input_label = np.array([1])  # Example labels (1 for foreground, 0 for background)

    # Predict masks using the points
    masks, scores, logits = predictor.predict_with_points(input_point, input_label)

    # Now, provide the input_labels along with the point_coords when calling display_results
    predictor.display_results(image, masks, scores, point_coords=input_point, input_labels=input_label)

    # Example: Predict masks based on a bounding box
    box_coords = np.array([50, 50, 400, 400])  # Example box coordinates [x0, y0, x1, y1]
    masks, scores, logits = predictor.predict_with_box(box_coords)

    # Display the results with the bounding box
    predictor.display_results(image, masks, scores, box_coords=box_coords)
