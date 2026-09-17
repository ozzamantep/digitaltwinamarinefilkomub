export const BT_STATUS = Object.freeze({ RUNNING: 'RUNNING', SUCCESS: 'SUCCESS', FAILURE: 'FAILURE' });

export class Condition {
  constructor(predicate) { this.predicate = predicate; }
  tick(context) { return this.predicate(context) ? BT_STATUS.SUCCESS : BT_STATUS.FAILURE; }
}

export class Action {
  constructor(handler) { this.handler = handler; }
  tick(context) { return this.handler(context); }
}

export class Sequence {
  constructor(children) { this.children = children; this.index = 0; }
  tick(context) {
    while (this.index < this.children.length) {
      const status = this.children[this.index].tick(context);
      if (status !== BT_STATUS.SUCCESS) return status;
      this.index++;
    }
    this.index = 0;
    return BT_STATUS.SUCCESS;
  }
}

export class Selector {
  constructor(children) { this.children = children; }
  tick(context) {
    for (const child of this.children) {
      const status = child.tick(context);
      if (status !== BT_STATUS.FAILURE) return status;
    }
    return BT_STATUS.FAILURE;
  }
}

export function createSafetyRecoveryTree() {
  return new Selector([
    new Action(({ command, safety, autonomy }) => {
      if (!safety?.floorBrake) return BT_STATUS.FAILURE;
      command.heave = autonomy.apply(command).heave;
      return BT_STATUS.RUNNING;
    }),
    new Action(({ command, safety, autonomy }) => {
      if (!safety?.autonomyRestricted) return BT_STATUS.FAILURE;
      Object.assign(command, autonomy.apply(command));
      return BT_STATUS.RUNNING;
    }),
    new Action(() => BT_STATUS.FAILURE),
  ]);
}