def test_use_window_context_is_exposed():
    import tpsprojector.gl_context as gc
    assert hasattr(gc, "use_window_context") and callable(gc.use_window_context)
