#!/usr/bin/env python3
"""
GO2 Voice Commander Node
Wake-word detection followed by Whisper-based speech transcription.
Parses intent from natural language and publishes structured commands.

Wake word: "hey robot" (or configurable)
Commands handled: come, stop, wait, follow, help
"""

import queue
import threading

import numpy as np
import pyaudio
import rclpy
import whisper
from rclpy.node import Node
from std_msgs.msg import String

WAKE_WORDS = {"hey robot", "hey go2", "excuse me", "come here", "come"}
COMMAND_MAP = {
    "come": "come here",
    "come here": "come here",
    "over here": "come here",
    "follow me": "follow",
    "follow": "follow",
    "stop": "stop",
    "wait": "stop",
    "stay": "stop",
    "help": "help",
    "help me": "help",
}


class VoiceCommanderNode(Node):
    def __init__(self):
        super().__init__("voice_commander_node")

        self.declare_parameter("whisper_model", "base.en")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("listen_duration_sec", 3.0)
        self.declare_parameter("energy_threshold", 800.0)

        model_name = self.get_parameter("whisper_model").value
        self.fs = self.get_parameter("sample_rate").value
        self.listen_duration = self.get_parameter("listen_duration_sec").value
        self.energy_threshold = self.get_parameter("energy_threshold").value

        self.get_logger().info(f"Loading Whisper model '{model_name}' ...")
        self.whisper_model = whisper.load_model(model_name)
        self.get_logger().info("Whisper model loaded.")

        # Publisher
        self.cmd_pub = self.create_publisher(String, "/go2/voice_command", 10)
        self.raw_pub = self.create_publisher(String, "/go2/voice_raw_transcript", 10)

        # Audio
        self.pa = pyaudio.PyAudio()
        self.audio_queue = queue.Queue()
        self.running = True

        self.listen_thread = threading.Thread(target=self._audio_listener, daemon=True)
        self.process_thread = threading.Thread(target=self._command_processor, daemon=True)
        self.listen_thread.start()
        self.process_thread.start()

        self.get_logger().info("VoiceCommanderNode ready. Listening for wake word...")

    def _audio_listener(self):
        """Continuously captures audio and enqueues chunks that exceed energy threshold."""
        chunk_size = int(self.fs * 0.1)  # 100ms chunks for VAD
        stream = self.pa.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.fs,
            input=True,
            frames_per_buffer=chunk_size,
        )

        buffer = []
        collecting = False
        collect_frames = int(self.listen_duration / 0.1)
        collected = 0

        while self.running:
            try:
                raw = stream.read(chunk_size, exception_on_overflow=False)
                chunk = np.frombuffer(raw, dtype=np.int16)
                energy = np.mean(chunk ** 2)

                if energy > self.energy_threshold and not collecting:
                    collecting = True
                    collected = 0
                    buffer = [chunk]
                elif collecting:
                    buffer.append(chunk)
                    collected += 1
                    if collected >= collect_frames:
                        audio_segment = np.concatenate(buffer).astype(np.float32) / 32768.0
                        self.audio_queue.put(audio_segment)
                        collecting = False
                        buffer = []
            except Exception as e:
                self.get_logger().warn(f"Audio listener error: {e}")

        stream.close()

    def _command_processor(self):
        """Transcribes audio segments and publishes parsed commands."""
        while self.running:
            try:
                audio = self.audio_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            try:
                result = self.whisper_model.transcribe(
                    audio,
                    language="en",
                    fp16=False,
                    condition_on_previous_text=False,
                )
                transcript = result["text"].strip().lower()

                self.get_logger().info(f"Transcript: '{transcript}'")

                # Publish raw transcript
                raw_msg = String()
                raw_msg.data = transcript
                self.raw_pub.publish(raw_msg)

                # Parse command
                command = self._parse_command(transcript)
                if command:
                    cmd_msg = String()
                    cmd_msg.data = command
                    self.cmd_pub.publish(cmd_msg)
                    self.get_logger().info(f"Command published: '{command}'")

            except Exception as e:
                self.get_logger().error(f"Transcription error: {e}")

    def _parse_command(self, transcript: str) -> str:
        """
        Maps transcript to a normalized command string.
        Returns None if no recognized command found.
        """
        # Direct match first
        for trigger, cmd in COMMAND_MAP.items():
            if trigger in transcript:
                return cmd

        # Check wake words — if present, treat as implicit "come here"
        for wake in WAKE_WORDS:
            if wake in transcript:
                return "come here"

        return None

    def destroy_node(self):
        self.running = False
        self.pa.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
