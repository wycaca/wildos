import torch

from explorfm.explorfm_model import ExploRFMInference, ModelPrecision


def test_image_forward_uses_inference_mode_without_changing_output():
    calls = []

    class Model:
        def __call__(self, value):
            calls.append(torch.is_inference_mode_enabled())
            return value * 2.0, value + 1.0, value - 1.0

    inference = object.__new__(ExploRFMInference)
    inference.device = "cpu"
    inference.model_precision = ModelPrecision.FP32
    inference.model = Model()
    source = torch.tensor([1.0, 2.0], requires_grad=True)

    outputs = inference.forward(source)

    assert calls == [True]
    assert torch.equal(outputs[0], torch.tensor([2.0, 4.0]))
    assert torch.equal(outputs[1], torch.tensor([2.0, 3.0]))
    assert torch.equal(outputs[2], torch.tensor([0.0, 1.0]))
    assert all(not output.requires_grad for output in outputs)


def test_fp16_forward_preserves_previous_input_conversion():
    received = []

    class Model:
        def __call__(self, value):
            received.append(value)
            return value, value, value

    inference = object.__new__(ExploRFMInference)
    inference.device = "cpu"
    inference.model_precision = ModelPrecision.FP16
    inference.model = Model()
    source = torch.tensor([1.1, 2.2], dtype=torch.float32)

    outputs = inference.forward(source)

    expected = source.to("cpu").half()
    assert torch.equal(received[0], expected)
    assert all(torch.equal(output, expected) for output in outputs)


def test_text_forward_uses_inference_mode():
    calls = []

    class Tokens:
        def to(self, device):
            return self

    class TextModel:
        tokenizer = lambda self, queries: Tokens()

        def encode_text(self, tokens, normalize):
            calls.append((torch.is_inference_mode_enabled(), normalize))
            return torch.tensor([[1.0, 0.0]])

    inference = object.__new__(ExploRFMInference)
    inference.device = "cpu"
    inference.text_model = TextModel()

    result = inference.forward_on_text(["chair"])

    assert calls == [(True, True)]
    assert torch.equal(result, torch.tensor([[1.0, 0.0]]))
