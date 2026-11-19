"""One model per process; models supply their own request/response schemas."""

import asyncio
import logging
from contextlib import asynccontextmanager
from functools import partial

from fastapi import FastAPI, HTTPException
from starlette.concurrency import run_in_threadpool


def create_app(factory, request_type, response_type, path="/infer",
               method="infer", extra_routes=()):
    """extra_routes: iterable of (path, request_type, response_type, method)
    served by the same predictor instance and busy slot (e.g. /infer_frames)."""
    @asynccontextmanager
    async def lifespan(app):
        app.state.predictor = await run_in_threadpool(factory)
        app.state.busy = False
        app.state.ready = True
        try:
            yield
        finally:
            app.state.ready = False
            close = getattr(app.state.predictor, "close", None)
            if close is not None:
                await run_in_threadpool(close)

    app = FastAPI(title="Aerohub model worker", lifespan=lifespan)
    app.state.ready = False

    @app.get("/health/live")
    async def live():
        return {"status": "alive"}

    @app.get("/health/ready")
    async def ready():
        if not app.state.ready:
            raise HTTPException(503, "model is not ready")
        return dict(app.state.predictor.metadata, busy=app.state.busy)

    async def infer(body, predictor_method_name):
        if not app.state.ready:
            raise HTTPException(503, "model is not ready")
        # No await between checking and acquiring: this runs on one event loop.
        if app.state.busy:
            raise HTTPException(429, "model is busy")
        app.state.busy = True

        async def execute():
            try:
                predictor_method = getattr(app.state.predictor, predictor_method_name)
                return await run_in_threadpool(
                    partial(predictor_method, body.model_dump())
                )
            except ValueError as error:
                raise HTTPException(422, str(error)) from error
            except Exception as error:
                logging.exception("Inference failed")
                raise HTTPException(500, "inference failed; see worker log") from error
            finally:
                app.state.busy = False

        # A disconnected/cancelled HTTP caller does not release an active GPU slot.
        task = asyncio.create_task(execute())
        task.add_done_callback(lambda result: result.exception() if not result.cancelled() else None)
        return await asyncio.shield(task)

    def register(route, input_type, output_type, predictor_method_name):
        async def handler(body):
            return await infer(body, predictor_method_name)
        handler.__annotations__["body"] = input_type
        app.post(route, response_model=output_type)(handler)

    register(path, request_type, response_type, method)
    for route in extra_routes:
        register(*route)
    return app
