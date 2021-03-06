#ifndef __NET_FIB_RULES_H
#define __NET_FIB_RULES_H

#include <linux/types.h>
#include <linux/netdevice.h>
#include <linux/fib_rules.h>
#include <net/flow.h>
#include <net/netlink.h>

struct fib_rule
{
	struct list_head	list;
	atomic_t		refcnt;
	int			ifindex;
	char			ifname[IFNAMSIZ];
	u32			pref;
	u32			flags;
	u32			table;
	u8			action;
	struct rcu_head		rcu;
};

struct fib_lookup_arg
{
	void			*lookup_ptr;
	void			*result;
	struct fib_rule		*rule;
};

struct fib_rules_ops
{
	int			family;
	struct list_head	list;
	int			rule_size;

	int			(*action)(struct fib_rule *,
					  struct flowi *, int,
					  struct fib_lookup_arg *);
	int			(*match)(struct fib_rule *,
					 struct flowi *, int);
	int			(*configure)(struct fib_rule *,
					     struct sk_buff *,
					     struct nlmsghdr *,
					     struct fib_rule_hdr *,
					     struct nlattr **);
	int			(*compare)(struct fib_rule *,
					   struct fib_rule_hdr *,
					   struct nlattr **);
	int			(*fill)(struct fib_rule *, struct sk_buff *,
					struct nlmsghdr *,
					struct fib_rule_hdr *);
	u32			(*default_pref)(void);

	int			nlgroup;
	struct nla_policy	*policy;
	struct list_head	*rules_list;
	struct module		*owner;
};

static inline void fib_rule_get(struct fib_rule *rule)
{
	atomic_inc(&rule->refcnt);
}

static inline void fib_rule_put_rcu(struct rcu_head *head)
{
	struct fib_rule *rule = container_of(head, struct fib_rule, rcu);
	kfree(rule);
}

static inline void fib_rule_put(struct fib_rule *rule)
{
	if (atomic_dec_and_test(&rule->refcnt))
		call_rcu(&rule->rcu, fib_rule_put_rcu);
}

extern int			fib_rules_register(struct fib_rules_ops *);
extern int			fib_rules_unregister(struct fib_rules_ops *);

extern int			fib_rules_lookup(struct fib_rules_ops *,
						 struct flowi *, int flags,
						 struct fib_lookup_arg *);

extern int			fib_nl_newrule(struct sk_buff *,
					       struct nlmsghdr *, void *);
extern int			fib_nl_delrule(struct sk_buff *,
					       struct nlmsghdr *, void *);
extern int			fib_rules_dump(struct sk_buff *,
					       struct netlink_callback *, int);
#endif


