#!/usr/bin/env python3
import queue
import threading

import nemo.collections.asr as nemo_asr
import numpy as np
import rclpy
import torch
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String


class NemoASRNode(Node):
    def __init__(self):
        super().__init__("nemo_asr_node")

        # Parameters
        self.declare_parameter("model_name", "nvidia/stt_en_fastconformer_ctc_small")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("buffer_size_ms", 1000) # 1 second buffer for now

        model_name = self.get_parameter("model_name").value
        self.fs = self.get_parameter("sample_rate").value
        self.buffer_size = int(self.fs * self.get_parameter("buffer_size_ms").value / 1000)

        # Load NeMo model
        self.get_logger().info(f"Loading NeMo model: {model_name}...")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.asr_model = nemo_asr.models.EncDecCTCModelBPE.from_pretrained(model_name)
        self.asr_model = self.asr_model.to(self.device)
        self.asr_model.eval()
        self.get_logger().info("NeMo model loaded successfully.")

        # Audio buffer
        self.audio_queue = queue.Queue()
        self.current_buffer = []

        # Publishers & Subscribers
        self.sub = self.create_subscription(
            Int16MultiArray, "/go2/audio/mono_raw", self.audio_callback, 10
        )
        self.pub = self.create_publisher(String, "/go2/audio/transcript", 10)

        # Processing thread
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.process_audio_loop)
        self.thread.start()

        self.get_logger().info("NemoASRNode started. Listening for mono audio.")

    def audio_callback(self, msg):
        self.audio_queue.put(np.array(msg.data, dtype=np.int16))

    def process_audio_loop(self):
        while not self.stop_event.is_set():
            try:
                # Get all available chunks from queue
                while not self.audio_queue.empty():
                    chunk = self.audio_queue.get_nowait()
                    self.current_buffer.extend(chunk)

                if len(self.current_buffer) >= self.buffer_size:
                    # Take exactly buffer_size
                    audio_segment = np.array(self.current_buffer[:self.buffer_size], dtype=np.float32) / 32768.0
                    # Shift buffer (simple overlap would be better, but let's start simple)
                    # Keep some context
                    overlap = int(self.buffer_size * 0.2)
                    self.current_buffer = self.current_buffer[self.buffer_size - overlap:]

                    # Inference
                    transcript = self.run_asr(audio_segment)
                    if transcript and len(transcript.strip()) > 0:
                        msg = String()
                        msg.data = transcript
                        self.pub.publish(msg)
                        self.get_logger().info(f"ASR: {transcript}")

                else:
                    # Wait a bit if not enough data
                    rclpy.spin_once(self, timeout_sec=0.1)

            except Exception as e:
                self.get_logger().error(f"Error in processing loop: {e}")

    def run_asr(self, audio_segment):
        with torch.no_grad():
            input_signal = torch.tensor(audio_segment).unsqueeze(0).to(self.device)
            input_signal_len = torch.tensor([len(audio_segment)]).to(self.device)

            # EncDecCTCModelBPE has transcribe method which is easier
            # but for streaming we might want manual forward if we had cache
            # Let's use a temporary file-less approach if possible or simple transcribe
            # Actually, transcribe() usually takes a list of paths.
            # For raw tensors, we can use forward() and decode.

            log_probs, encoded_len, greedy_predictions = self.asr_model(
                input_signal=input_signal, input_signal_length=input_signal_len
            )

            hypotheses = self.asr_model.decoding.ctc_decoder_predictions_tensor(
                greedy_predictions, decoder_lengths=encoded_len
            )
            return hypotheses[0] if hypotheses else ""

    def destroy_node(self):
        self.stop_event.set()
        self.thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = NemoASRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
