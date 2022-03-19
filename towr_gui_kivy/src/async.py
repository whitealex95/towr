import asyncio
from codetiming import Timer

async def task(name, work_queue):
    timer = Timer(text=f"Task {name} elapsed time: {{:.1f}}")
    while not work_queue.empty():
        delay = await work_queue.get()
        print(f"Task {name} running")
        timer.start()
        await asyncio.sleep(delay)
        timer.stop()

async def task_print(name, msg_queue):
  timer = Timer(text=f"Task {name} elapsed time: {{:.1f}}")
  pub_msgs = [1, 2, 3, 4, 5]
  while not msg_queue.empty():
    print(f"Task {name} running")
    print(pub_msgs)
    timer.start()
    await asyncio.sleep(1)
    timer.stop()

async def main():
    """
    This is the main entry point for the program
    """
    # Create the queue of work
    work_queue = asyncio.Queue()

    # Put some work in the queue
    for work in [2, 1, 1, 2.1]:
        await work_queue.put(work)
    await task_print('hi')
    # Run the tasks
    with Timer(text="\nTotal elapsed time: {:.1f}"):
        await asyncio.gather(
            asyncio.create_task(task("One", work_queue)),
            # asyncio.create_task(task("Two", work_queue)),
            asyncio.create_task(task_print("Three")),
        )

if __name__ == "__main__":
    asyncio.run(main())
