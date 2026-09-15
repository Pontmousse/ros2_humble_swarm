
You’re not overcomplicating it. What you described is actually a good smoke-test pattern for an Ansible fleet.
I would make it slightly more systematic:

    Connectivity test

ansible all -m ping

This tells you SSH + Python + Ansible access are working.

    Deployment test playbook

Write one tiny marker file on every Pi:

- name: Fleet deployment smoke test
  hosts: robots
  become: true

  vars:
    test_version: "v3"

  tasks:
    - name: Write deployment marker
      ansible.builtin.copy:
        content: "{{ test_version }}\n"
        dest: /tmp/ansible_deploy_test.txt
        mode: "0644"

The resulting file is just:

v3

Then run:

ansible-playbook test_deploy.yml

    Independent validation playbook

Yes, I like your idea of having validation separate from deployment.

- name: Validate fleet deployment
  hosts: robots
  gather_facts: false

  vars:
    expected_version: "v3"

  tasks:
    - name: Read deployment marker
      ansible.builtin.slurp:
        src: /tmp/ansible_deploy_test.txt
      register: marker

    - name: Verify correct version
      ansible.builtin.assert:
        that:
          - marker.content | b64decode | trim == expected_version
        fail_msg: >
          Wrong deployment version on {{ inventory_hostname }}
        success_msg: >
          {{ inventory_hostname }} is running {{ expected_version }}

Then:

ansible-playbook validate.yml

That gives you a proper success/failure result for every machine.
The version marker is a very good idea because merely checking:

Does file exist?

doesn't prove that the latest deployment happened.
Checking:

file contents == expected deployment version

does.
For example:

robot01    v3   PASS
robot02    v3   PASS
robot03    v2   FAIL
robot04    v3   PASS

and you immediately know robot03 missed the latest deployment.
For a real stress test, I'd test a few more things beyond just the marker file.
Run the same deployment repeatedly:

ansible-playbook test_deploy.yml
ansible-playbook test_deploy.yml
ansible-playbook test_deploy.yml

On the second and third runs, ideally you should see:

changed=0
failed=0

That is testing idempotency, which is one of the most important things to verify in an Ansible workflow.
Then test your whole fleet simultaneously:

ansible-playbook test_deploy.yml --forks 20

assuming you have around that many hosts.
You can also deliberately make one Pi unavailable. Turn one off or disconnect it and run the playbook. You want to verify that:

robot01 → success
robot02 → success
robot03 → UNREACHABLE
robot04 → success

rather than one dead robot somehow disrupting the entire deployment.
For your actual RoboMaster deployment, I would eventually evolve the marker into something like:

deployment_version=v17
image=robomaster-control:v17
git_commit=8ac41f2
deployed_at=2026-09-14T19:20:00

But for testing the Ansible infrastructure itself, I would absolutely start with your proposed single-line version file.
Something as simple as:

robomaster-test-v4

is enough.
Your test progression would then be:

ansible ping
     ↓
deploy v1 marker
     ↓
validate v1
     ↓
deploy v2 marker
     ↓
validate v2
     ↓
run deployment again
     ↓
expect changed=0
     ↓
disconnect one Pi
     ↓
verify failure isolation
     ↓
reconnect it
     ↓
redeploy whole fleet
     ↓
validate everyone is v2

That is actually a pretty strong test of your Ansible infrastructure before you trust it with Docker, ROS2, systemd services, and RoboMaster experiments.
One extra thing I would add: use a timestamp only for informational purposes, not validation. If you put the current time into the file, Ansible will change it on every execution, and you lose your idempotency test. A fixed version such as v4 is much better.

